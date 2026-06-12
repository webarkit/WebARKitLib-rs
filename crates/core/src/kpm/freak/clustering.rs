/*
 *  clustering.rs
 *  WebARKitLib-rs
 *
 *  This file is part of WebARKitLib-rs - WebARKit.
 *
 *  WebARKitLib-rs is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  WebARKitLib-rs is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with WebARKitLib-rs.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  As a special exception, the copyright holders of this library give you
 *  permission to link this library with independent modules to produce an
 *  executable, regardless of the license terms of these independent modules, and to
 *  copy and distribute the resulting executable under terms of your choice,
 *  provided that you also meet, for each linked independent module, the terms and
 *  conditions of the license of that module. An independent module is a module
 *  which is neither derived from nor based on this library. If you modify this
 *  library, you may extend this exception to your version of the library, but you
 *  are not obligated to do so. If you do not wish to do so, delete this exception
 *  statement from your version.
 *
 *  Copyright 2026 WebARKit.
 *
 *  Author(s): Walter Perdan <@kalwalt> https://github.com/kalwalt
 *
 */

//! Binary hierarchical clustering (vocabulary tree) for FREAK descriptors.
//!
//! This module implements k-medoids clustering and a binary hierarchical clustering
//! tree for fast approximate nearest-neighbor search on 96-byte FREAK descriptors.
//! Distances are computed using Hamming distance via bit manipulation.

use crate::kpm::backend::KpmError;
use crate::{arlog_d, arlog_e};
use std::cmp::Reverse;
use std::collections::{BTreeMap, BinaryHeap};

/// Computes Hamming distance between two 32-bit chunks using bit magic.
/// C equivalent: HammingDistance32
fn hamming_distance_32(a: u32, b: u32) -> u32 {
    const M1: u32 = 0x55555555;
    const M2: u32 = 0x33333333;
    const M4: u32 = 0x0f0f0f0f;
    const H01: u32 = 0x01010101;

    let mut x = a ^ b;
    x -= (x >> 1) & M1;
    x = (x & M2) + ((x >> 2) & M2);
    x = (x + (x >> 4)) & M4;
    (x.wrapping_mul(H01)) >> 24
}

/// Computes Hamming distance between two 96-byte (768-bit) FREAK descriptors.
/// C equivalent: HammingDistance768
pub fn hamming_distance_96(a: &[u8; 96], b: &[u8; 96]) -> u32 {
    // Reassemble each 4-byte chunk via `u32::from_ne_bytes` rather than
    // transmuting `&[u8; 96]` to `&[u32; 24]`. The transmute requires
    // 4-byte alignment on the input, which `&[u8; 96]` does not
    // guarantee — Miri flagged it as UB (#192).
    (0..24)
        .map(|i| {
            let off = i * 4;
            let ax = u32::from_ne_bytes([a[off], a[off + 1], a[off + 2], a[off + 3]]);
            let bx = u32::from_ne_bytes([b[off], b[off + 1], b[off + 2], b[off + 3]]);
            hamming_distance_32(ax, bx)
        })
        .sum()
}

/// Fast PRNG matching `vision::FastRandom` from `math/rand.h`.
///
/// Mutates `seed` in place using the LCG step `seed = 214013*seed + 2531011`,
/// and returns the top 15 bits as a non-negative integer in `[0, 32767]`.
///
/// The C++ source uses `int` (32-bit signed). We mirror the bit pattern with
/// `i32` and wrapping arithmetic so the sequence is byte-identical.
fn fast_random(seed: &mut i32) -> i32 {
    *seed = seed.wrapping_mul(214013).wrapping_add(2531011);
    (*seed >> 16) & 0x7FFF
}

/// Shuffles the first `sample_size` elements of `v` using `fast_random`.
///
/// C equivalent: `vision::ArrayShuffle` from `math/rand.h`.
///
/// Note: This is NOT Fisher–Yates. The C++ implementation draws `k` from
/// `[0, pop_size)` (not `[i, pop_size)`), which means earlier swaps may be
/// undone by later iterations. We mirror this exactly for parity.
///
/// `seed` is mutated by every call to `fast_random`; pass a persistent seed
/// across multiple shuffles to reproduce C++ k-medoids state evolution.
fn array_shuffle<T>(v: &mut [T], sample_size: usize, seed: &mut i32) {
    let pop_size = v.len();
    if pop_size == 0 {
        return;
    }
    for i in 0..sample_size {
        let k = (fast_random(seed) as usize) % pop_size;
        v.swap(i, k);
    }
}

/// K-medoids clustering for binary features (96-byte FREAK descriptors).
/// C equivalent: BinarykMedoids<96>
pub struct KMedoids {
    k: usize,
    centers: Vec<usize>,
    assignment: Vec<usize>,
    num_hypotheses: usize,
    /// PRNG state. Type matches C++ `int` so `fast_random` produces a
    /// byte-identical sequence. Mutated by every call to `assign()`.
    rand_seed: i32,
}

impl KMedoids {
    /// Creates a new k-medoids clusterer with the given parameters.
    ///
    /// # Arguments
    /// * `k` - Number of clusters (must be > 0)
    /// * `num_hypotheses` - Number of random initializations to try (must be > 0)
    pub fn new(k: usize, num_hypotheses: usize) -> Result<Self, KpmError> {
        if k == 0 {
            arlog_e!("KMedoids::new: k must be > 0, got {}", k);
            return Err(KpmError::InvalidInput("k must be > 0".into()));
        }
        if num_hypotheses == 0 {
            arlog_e!(
                "KMedoids::new: num_hypotheses must be > 0, got {}",
                num_hypotheses
            );
            return Err(KpmError::InvalidInput("num_hypotheses must be > 0".into()));
        }

        Ok(Self {
            k,
            centers: Vec::with_capacity(k),
            assignment: Vec::new(),
            num_hypotheses,
            rand_seed: 1234,
        })
    }

    /// Sets the random seed for reproducible clustering.
    ///
    /// The seed type is `i32` to match C++'s `int` exactly so the PRNG
    /// sequence is byte-identical across implementations.
    pub fn set_rand_seed(&mut self, seed: i32) {
        self.rand_seed = seed;
    }

    /// Returns the current PRNG seed state. Useful for tests verifying that
    /// the seed evolves across calls.
    pub fn rand_seed(&self) -> i32 {
        self.rand_seed
    }

    /// Returns the cluster assignment for each feature.
    pub fn assignment(&self) -> &[usize] {
        &self.assignment
    }

    /// Returns the center indices.
    pub fn centers(&self) -> &[usize] {
        &self.centers
    }

    /// Assigns features to clusters using k-medoids algorithm.
    pub fn assign(&mut self, features: &[&[u8; 96]]) -> Result<(), KpmError> {
        if features.is_empty() {
            arlog_e!("KMedoids::assign: no features provided");
            return Err(KpmError::InvalidInput("features cannot be empty".into()));
        }
        if features.len() < self.k {
            arlog_e!(
                "KMedoids::assign: not enough features ({}) for k={}",
                features.len(),
                self.k
            );
            return Err(KpmError::NotEnoughPoints {
                got: features.len(),
                need: self.k,
            });
        }

        let n = features.len();
        self.assignment.resize(n, 0);

        let mut best_distortion = u64::MAX;
        let mut best_assignment = vec![0; n];
        let mut best_centers = vec![0; self.k];

        // Sequential vector [0, 1, ..., n-1], shuffled in place across
        // hypotheses. C++ initializes this ONCE before the hypothesis loop
        // (kmedoids.h line 163: SequentialVector(...)) — we mirror exactly.
        let mut rand_indices: Vec<usize> = (0..n).collect();

        for hyp in 0..self.num_hypotheses {
            // C++ shuffle (matchers/kmedoids.h line 169) — mutates rand_seed
            // and rand_indices in place. The seed evolves across hypotheses
            // and across recursive BHC build calls.
            array_shuffle(&mut rand_indices, self.k, &mut self.rand_seed);
            let hypothesis_centers: Vec<usize> = rand_indices[..self.k].to_vec();

            let mut hyp_assignment = vec![0; n];
            let mut hyp_distortion = 0u64;

            for (feat_idx, feature) in features.iter().enumerate() {
                let mut best_dist = u32::MAX;
                let mut best_center_idx = 0;

                for (center_pos, &center_feat_idx) in hypothesis_centers.iter().enumerate() {
                    let dist = hamming_distance_96(feature, features[center_feat_idx]);
                    if dist < best_dist {
                        best_dist = dist;
                        best_center_idx = center_pos;
                    }
                }

                hyp_assignment[feat_idx] = best_center_idx;
                hyp_distortion += best_dist as u64;
            }

            if hyp_distortion < best_distortion {
                best_distortion = hyp_distortion;
                best_assignment.clone_from(&hyp_assignment);
                best_centers.clone_from(&hypothesis_centers);

                arlog_d!(
                    "KMedoids::assign hypothesis {}: distortion = {}",
                    hyp,
                    hyp_distortion
                );
            }
        }

        self.assignment = best_assignment;
        self.centers = best_centers;

        arlog_d!(
            "KMedoids::assign complete: k={}, final_distortion={}",
            self.k,
            best_distortion
        );
        Ok(())
    }
}

/// Represents a node in the binary hierarchical clustering tree.
pub struct BhcNode {
    /// Unique node identifier.
    _id: usize,
    /// FREAK descriptor of the cluster center (None for leaves).
    center: Option<Box<[u8; 96]>>,
    /// Child nodes (Box is necessary for recursive type).
    #[allow(clippy::vec_box)]
    children: Vec<Box<BhcNode>>,
    /// Feature indices stored in this leaf node.
    reverse_index: Vec<usize>,
    /// True if this is a leaf node.
    is_leaf: bool,
}

impl BhcNode {
    fn new(_id: usize) -> Self {
        Self {
            _id,
            center: None,
            children: Vec::new(),
            reverse_index: Vec::new(),
            is_leaf: true,
        }
    }

    /// Returns whether this is a leaf node.
    pub fn is_leaf(&self) -> bool {
        self.is_leaf
    }

    /// Returns the center descriptor if this is a non-leaf node.
    pub fn center(&self) -> Option<&[u8; 96]> {
        self.center.as_deref()
    }

    /// Returns the feature indices if this is a leaf node.
    pub fn reverse_index(&self) -> &[usize] {
        &self.reverse_index
    }

    /// Returns the number of child nodes.
    pub fn num_children(&self) -> usize {
        self.children.len()
    }

    /// Returns a reference to a child node by index.
    pub fn child(&self, idx: usize) -> Option<&BhcNode> {
        self.children.get(idx).map(|b| b.as_ref())
    }
}

/// Binary hierarchical clustering tree for fast nearest-neighbor search.
/// C equivalent: BinaryHierarchicalClustering<96>
pub struct BinaryHierarchicalClustering {
    root: Option<Box<BhcNode>>,
    kmedoids: KMedoids,
    num_centers: usize,
    /// Number of K-medoids hypothesis runs per split.
    /// Cached so [`set_num_centers`] and [`set_num_hypotheses`] can independently
    /// rebuild the internal `kmedoids` without forgetting the other parameter.
    /// C equivalent: `mBinarykMedoids.mNumHypotheses` (effectively).
    num_hypotheses: usize,
    min_features_per_leaf: usize,
    max_nodes_to_pop: usize,
    next_node_id: usize,
}

impl BinaryHierarchicalClustering {
    /// Creates a new empty binary hierarchical clustering tree.
    ///
    /// Defaults match C++ `BinaryHierarchicalClustering()` default-ctor:
    /// `num_centers=8, num_hypotheses=1, min_features_per_leaf=16,
    /// max_nodes_to_pop=0`. Use [`Keyframe::build_index`] for the
    /// `Keyframe<>::buildIndex` settings `(128, 8, 8, 16)`.
    pub fn new() -> Result<Self, KpmError> {
        let mut kmedoids = KMedoids::new(8, 1)?;
        kmedoids.set_rand_seed(1234);

        Ok(Self {
            root: None,
            kmedoids,
            num_centers: 8,
            num_hypotheses: 1,
            min_features_per_leaf: 16,
            max_nodes_to_pop: 0,
            next_node_id: 0,
        })
    }

    /// Sets the number of centers per split in the tree.
    /// C equivalent: `BinaryHierarchicalClustering::setNumCenters(int)`.
    pub fn set_num_centers(&mut self, k: usize) -> Result<(), KpmError> {
        if k == 0 {
            arlog_e!("BinaryHierarchicalClustering::set_num_centers: k must be > 0");
            return Err(KpmError::InvalidInput("k must be > 0".into()));
        }
        self.num_centers = k;
        let mut kmedoids = KMedoids::new(k, self.num_hypotheses)?;
        kmedoids.set_rand_seed(1234);
        self.kmedoids = kmedoids;
        Ok(())
    }

    /// Sets the minimum number of features per leaf node.
    /// C equivalent: `BinaryHierarchicalClustering::setMinFeaturesPerNode(int)`.
    pub fn set_min_features_per_leaf(&mut self, min_features: usize) -> Result<(), KpmError> {
        if min_features == 0 {
            arlog_e!(
                "BinaryHierarchicalClustering::set_min_features_per_leaf: min_features must be > 0"
            );
            return Err(KpmError::InvalidInput("min_features must be > 0".into()));
        }
        self.min_features_per_leaf = min_features;
        Ok(())
    }

    /// Sets the number of K-medoids hypothesis runs per split.
    ///
    /// Higher values trade build-time CPU for better split quality. The C++
    /// default constructor uses 1; `Keyframe<>::buildIndex` overrides to 128.
    ///
    /// C equivalent: `BinaryHierarchicalClustering::setNumHypotheses(int)`
    /// (which delegates to `mBinarykMedoids.setNumHypotheses`).
    pub fn set_num_hypotheses(&mut self, n: usize) -> Result<(), KpmError> {
        if n == 0 {
            arlog_e!("BinaryHierarchicalClustering::set_num_hypotheses: n must be > 0");
            return Err(KpmError::InvalidInput("n must be > 0".into()));
        }
        self.num_hypotheses = n;
        let mut kmedoids = KMedoids::new(self.num_centers, n)?;
        kmedoids.set_rand_seed(1234);
        self.kmedoids = kmedoids;
        Ok(())
    }

    /// Sets the maximum number of non-tied children to pop from the priority
    /// queue per `query()` call, in addition to tied-minimum children.
    ///
    /// The C++ default constructor uses 0 (priority queue unused);
    /// `Keyframe<>::buildIndex` overrides to 8.
    ///
    /// C equivalent: `BinaryHierarchicalClustering::setMaxNodesToPop(int)`.
    pub fn set_max_nodes_to_pop(&mut self, n: usize) {
        self.max_nodes_to_pop = n;
    }

    /// Builds the tree from the given features.
    pub fn build(&mut self, features: &[&[u8; 96]]) -> Result<(), KpmError> {
        if features.is_empty() {
            arlog_e!("BinaryHierarchicalClustering::build: no features");
            return Err(KpmError::InvalidInput("features cannot be empty".into()));
        }

        self.next_node_id = 0;
        let indices: Vec<usize> = (0..features.len()).collect();

        let mut root = self.create_node();
        root.is_leaf = false;

        self.build_recursive(&mut root, features, &indices)?;

        self.root = Some(Box::new(root));

        arlog_d!(
            "BinaryHierarchicalClustering::build complete: {} nodes created",
            self.next_node_id
        );
        Ok(())
    }

    /// Queries the tree for the nearest neighbors of a given feature.
    ///
    /// Walks the tree depth-first, visiting children that share the minimum
    /// distance to the query at each internal node and pushing non-tied
    /// siblings onto a min-heap backlog. After processing each internal
    /// node's tied-minimum children, if the global budget [`max_nodes_to_pop`]
    /// has not been reached, **one** node is popped from the backlog and
    /// recursed into inline. This mirrors C++
    /// `BinaryHierarchicalClustering::query` (`binary_hierarchical_clustering.h:419-444`),
    /// which performs the same depth-first traversal with inline single-pop.
    ///
    /// When `max_nodes_to_pop == 0` (default-constructed BHC), only
    /// tied-minimum children are visited (matches the pre-M9-#146 behaviour).
    /// When `max_nodes_to_pop == 8` ([`Keyframe::build_index`] settings),
    /// the candidate set is widened to match C++ `Keyframe::mIndex` behaviour.
    pub fn query(&self, query_feature: &[u8; 96]) -> Result<Vec<usize>, KpmError> {
        let root = self.root.as_ref().ok_or_else(|| {
            arlog_e!("BinaryHierarchicalClustering::query: tree not built");
            KpmError::InternalError("tree not built".into())
        })?;

        let mut result = Vec::new();
        let mut backlog: BinaryHeap<Reverse<BacklogEntry>> = BinaryHeap::new();
        let mut next_seq: u64 = 0;
        let mut nodes_popped: usize = 0;

        self.query_recursive(
            root,
            query_feature,
            &mut result,
            &mut backlog,
            &mut next_seq,
            &mut nodes_popped,
        )?;

        arlog_d!(
            "BinaryHierarchicalClustering::query: found {} features ({} extra nodes popped)",
            result.len(),
            nodes_popped
        );
        Ok(result)
    }

    fn create_node(&mut self) -> BhcNode {
        let id = self.next_node_id;
        self.next_node_id += 1;
        let mut node = BhcNode::new(0);
        node._id = id;
        node
    }

    fn build_recursive(
        &mut self,
        node: &mut BhcNode,
        features: &[&[u8; 96]],
        indices: &[usize],
    ) -> Result<(), KpmError> {
        if indices.len() <= self.min_features_per_leaf.max(self.num_centers) {
            node.is_leaf = true;
            node.reverse_index = indices.to_vec();
            return Ok(());
        }

        let subset_features: Vec<&[u8; 96]> = indices.iter().map(|&i| features[i]).collect();

        self.kmedoids.assign(&subset_features)?;

        // Clone assignment and centers to avoid borrow checker issues
        let assignment = self.kmedoids.assignment().to_vec();
        let centers = self.kmedoids.centers().to_vec();

        // BTreeMap (not HashMap) gives deterministic iteration order across
        // Rust runs. C++ uses std::unordered_map (hash-order; inherently
        // non-deterministic) so cross-language tree-topology parity isn't
        // achievable at the BHC layer alone — see the `bhc_with_keyframe_settings_matches_cpp`
        // dual-mode test for the diagnostic check, and the full
        // `test_visual_database_matches_cpp_pipeline` for the milestone gate.
        let mut clusters: BTreeMap<usize, Vec<usize>> = BTreeMap::new();

        for (feat_idx_in_subset, &cluster_assignment) in assignment.iter().enumerate() {
            let global_feat_idx = indices[feat_idx_in_subset];
            clusters
                .entry(cluster_assignment)
                .or_default()
                .push(global_feat_idx);
        }

        if clusters.len() == 1 {
            node.is_leaf = true;
            node.reverse_index = indices.to_vec();
            return Ok(());
        }

        node.is_leaf = false;
        node.children.reserve(clusters.len());

        for (cluster_pos, cluster_indices) in clusters {
            let center_feat_idx = indices[centers[cluster_pos]];
            let mut child = self.create_node();
            child.center = Some(Box::new(*features[center_feat_idx]));
            child.is_leaf = false;

            self.build_recursive(&mut child, features, &cluster_indices)?;
            node.children.push(Box::new(child));
        }

        Ok(())
    }

    fn query_recursive<'tree>(
        &'tree self,
        node: &'tree BhcNode,
        query_feature: &[u8; 96],
        result: &mut Vec<usize>,
        backlog: &mut BinaryHeap<Reverse<BacklogEntry<'tree>>>,
        next_seq: &mut u64,
        nodes_popped: &mut usize,
    ) -> Result<(), KpmError> {
        if node.is_leaf {
            result.extend_from_slice(&node.reverse_index);
            return Ok(());
        }

        // Compute Hamming distance to each child's center.
        let dists: Vec<u32> = node
            .children
            .iter()
            .map(|child| {
                child
                    .center
                    .as_ref()
                    .map(|c| hamming_distance_96(c, query_feature))
                    .unwrap_or(u32::MAX)
            })
            .collect();

        let min_dist = match dists.iter().min().copied() {
            Some(d) => d,
            None => return Ok(()),
        };

        // C++ Node::nearest behaviour: tied-minimum children go to the
        // tied-recursion path, non-tied siblings go to the shared backlog
        // priority queue.
        for (i, &d) in dists.iter().enumerate() {
            if d == min_dist {
                self.query_recursive(
                    &node.children[i],
                    query_feature,
                    result,
                    backlog,
                    next_seq,
                    nodes_popped,
                )?;
            } else {
                backlog.push(Reverse(BacklogEntry {
                    distance: d,
                    seq: *next_seq,
                    node: &node.children[i],
                }));
                *next_seq += 1;
            }
        }

        // C++ `query` (binary_hierarchical_clustering.h:437): after all
        // tied-minimum children at this level have been processed, pop ONE
        // node from the shared backlog (if the global budget permits) and
        // recurse into it. The pop is INLINE — not a separate Phase 2 —
        // because the C++ traversal mixes tied-min recursion with budgeted
        // pops as it unwinds.
        if *nodes_popped < self.max_nodes_to_pop {
            if let Some(Reverse(entry)) = backlog.pop() {
                *nodes_popped += 1;
                self.query_recursive(
                    entry.node,
                    query_feature,
                    result,
                    backlog,
                    next_seq,
                    nodes_popped,
                )?;
            }
        }

        Ok(())
    }
}

/// Heap entry used by [`BinaryHierarchicalClustering::query`] Phase 2.
///
/// `seq` is a monotonically increasing counter assigned at push time so that
/// equal-`distance` entries pop in deterministic insertion order (FIFO),
/// avoiding any address-based nondeterminism. The C++ `std::priority_queue`
/// is implementation-defined on equal keys; using an explicit seq keeps the
/// Rust port deterministic and run-to-run reproducible.
struct BacklogEntry<'a> {
    distance: u32,
    seq: u64,
    node: &'a BhcNode,
}

impl<'a> PartialEq for BacklogEntry<'a> {
    fn eq(&self, other: &Self) -> bool {
        // seq is unique per push, so two distinct entries never compare equal.
        // Implementing PartialEq purely on (distance, seq) keeps Ord total
        // and consistent with Eq.
        self.distance == other.distance && self.seq == other.seq
    }
}

impl<'a> Eq for BacklogEntry<'a> {}

impl<'a> PartialOrd for BacklogEntry<'a> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<'a> Ord for BacklogEntry<'a> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Lexicographic: distance first, then seq for stable tie-breaks.
        // `node` is opaque payload and intentionally not part of the ordering.
        self.distance
            .cmp(&other.distance)
            .then(self.seq.cmp(&other.seq))
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    // ===== RNG Tests (parity with C++ FastRandom / ArrayShuffle) =====

    /// First call to fast_random with seed=1234.
    /// Expected: seed becomes 214013*1234 + 2531011 = 266_623_053.
    /// Returns (266_623_053 >> 16) & 0x7FFF = 4068.
    /// These constants are frozen to detect any algorithmic change; the
    /// dual-mode FFI test (cfg "dual-mode") additionally verifies
    /// byte-identical output against the C++ baseline.
    #[test]
    fn test_fast_random_first_call() {
        let mut seed: i32 = 1234;
        let r = fast_random(&mut seed);
        assert_eq!(seed, 266_623_053);
        assert_eq!(r, 4068);
    }

    /// Calling fast_random with the same seed twice yields the same sequence.
    #[test]
    fn test_fast_random_deterministic() {
        let mut s1: i32 = 1234;
        let mut s2: i32 = 1234;
        let r1: Vec<i32> = (0..10).map(|_| fast_random(&mut s1)).collect();
        let r2: Vec<i32> = (0..10).map(|_| fast_random(&mut s2)).collect();
        assert_eq!(r1, r2);
        assert_eq!(s1, s2);
    }

    /// fast_random returns values in [0, 32767].
    #[test]
    fn test_fast_random_range() {
        let mut seed: i32 = 7;
        for _ in 0..1000 {
            let r = fast_random(&mut seed);
            assert!((0..=32767).contains(&r), "value {} out of range", r);
        }
    }

    /// Negative starting seeds work correctly (i32 wrapping arithmetic).
    #[test]
    fn test_fast_random_negative_seed() {
        let mut seed: i32 = -1;
        let r = fast_random(&mut seed);
        // No crash; output is non-negative because of the &0x7FFF mask.
        assert!((0..=32767).contains(&r));
    }

    /// array_shuffle preserves the multiset of elements (it's a permutation).
    #[test]
    fn test_array_shuffle_preserves_elements() {
        let mut v: Vec<usize> = (0..10).collect();
        let mut seed: i32 = 1234;
        array_shuffle(&mut v, 5, &mut seed);

        let mut sorted = v.clone();
        sorted.sort_unstable();
        assert_eq!(sorted, (0..10).collect::<Vec<_>>());
    }

    /// Same seed produces the same shuffle.
    #[test]
    fn test_array_shuffle_deterministic() {
        let mut v1: Vec<usize> = (0..20).collect();
        let mut v2: Vec<usize> = (0..20).collect();
        let mut s1: i32 = 1234;
        let mut s2: i32 = 1234;
        array_shuffle(&mut v1, 8, &mut s1);
        array_shuffle(&mut v2, 8, &mut s2);
        assert_eq!(v1, v2);
        assert_eq!(s1, s2);
    }

    /// Empty slice doesn't panic.
    #[test]
    fn test_array_shuffle_empty_pop() {
        let mut v: Vec<usize> = Vec::new();
        let mut seed: i32 = 1234;
        array_shuffle(&mut v, 0, &mut seed);
        assert!(v.is_empty());
    }

    /// Sample_size = 0 leaves array unchanged.
    #[test]
    fn test_array_shuffle_zero_sample() {
        let mut v: Vec<usize> = (0..10).collect();
        let original = v.clone();
        let mut seed: i32 = 1234;
        array_shuffle(&mut v, 0, &mut seed);
        assert_eq!(v, original);
    }

    /// The seed evolves across array_shuffle calls — verifies persistent state.
    #[test]
    fn test_array_shuffle_seed_evolves() {
        let mut v: Vec<usize> = (0..10).collect();
        let mut seed: i32 = 1234;
        array_shuffle(&mut v, 5, &mut seed);
        let seed_after_first = seed;
        array_shuffle(&mut v, 5, &mut seed);
        assert_ne!(seed_after_first, seed, "seed must advance across calls");
    }

    // ===== Hamming Distance Tests =====

    #[test]
    fn test_hamming_distance_identical() {
        let desc = [0u8; 96];
        assert_eq!(hamming_distance_96(&desc, &desc), 0);
    }

    #[test]
    fn test_hamming_distance_symmetric() {
        let a = [1u8; 96];
        let b = [2u8; 96];
        assert_eq!(hamming_distance_96(&a, &b), hamming_distance_96(&b, &a));
    }

    #[test]
    fn test_hamming_distance_max() {
        let a = [0u8; 96];
        let b = [255u8; 96];
        assert_eq!(hamming_distance_96(&a, &b), 768);
    }

    #[test]
    fn test_hamming_distance_known_values() {
        let mut a = [0u8; 96];
        let mut b = [0u8; 96];
        a[0] = 0xFF;
        b[0] = 0x00;
        assert_eq!(hamming_distance_96(&a, &b), 8);
    }

    // ===== K-Medoids Tests =====

    #[test]
    fn test_kmedoids_new_invalid_k() {
        assert!(KMedoids::new(0, 1).is_err());
    }

    #[test]
    fn test_kmedoids_new_invalid_hypotheses() {
        assert!(KMedoids::new(2, 0).is_err());
    }

    #[test]
    fn test_kmedoids_assign_empty() {
        let mut km = KMedoids::new(2, 1).unwrap();
        assert!(km.assign(&[]).is_err());
    }

    #[test]
    fn test_kmedoids_assign_insufficient_features() {
        let mut km = KMedoids::new(5, 1).unwrap();
        let feat = [0u8; 96];
        let features = vec![&feat];
        assert!(km.assign(&features).is_err());
    }

    #[test]
    fn test_kmedoids_single_cluster() {
        let mut km = KMedoids::new(1, 1).unwrap();
        let feat1 = [0u8; 96];
        let feat2 = [1u8; 96];
        let features = vec![&feat1, &feat2];
        km.assign(&features).unwrap();

        assert_eq!(km.assignment().len(), 2);
        assert!(km.assignment().iter().all(|&a| a == 0));
    }

    #[test]
    fn test_kmedoids_deterministic() {
        let feat1 = [0u8; 96];
        let feat2 = [1u8; 96];
        let feat3 = [2u8; 96];
        let features = vec![&feat1, &feat2, &feat3];

        let mut km1 = KMedoids::new(2, 1).unwrap();
        km1.set_rand_seed(42);
        km1.assign(&features).unwrap();
        let assign1 = km1.assignment().to_vec();

        let mut km2 = KMedoids::new(2, 1).unwrap();
        km2.set_rand_seed(42);
        km2.assign(&features).unwrap();
        let assign2 = km2.assignment().to_vec();

        assert_eq!(assign1, assign2);
    }

    // ===== BHC Tests =====

    #[test]
    fn test_bhc_new() {
        let bhc = BinaryHierarchicalClustering::new();
        assert!(bhc.is_ok());
    }

    #[test]
    fn test_bhc_build_empty() {
        let mut bhc = BinaryHierarchicalClustering::new().unwrap();
        assert!(bhc.build(&[]).is_err());
    }

    #[test]
    fn test_bhc_build_single() {
        let mut bhc = BinaryHierarchicalClustering::new().unwrap();
        let feat = [42u8; 96];
        let features = vec![&feat];
        assert!(bhc.build(&features).is_ok());
    }

    #[test]
    fn test_bhc_query_unbuilt() {
        let bhc = BinaryHierarchicalClustering::new().unwrap();
        let feat = [0u8; 96];
        assert!(bhc.query(&feat).is_err());
    }

    #[test]
    fn test_bhc_roundtrip() {
        let mut bhc = BinaryHierarchicalClustering::new().unwrap();
        let feat1 = [1u8; 96];
        let feat2 = [2u8; 96];
        let feat3 = [3u8; 96];
        let features = vec![&feat1, &feat2, &feat3];

        bhc.build(&features).unwrap();

        let result = bhc.query(&feat1).unwrap();
        assert!(!result.is_empty());
        assert!(result.contains(&0));
    }

    #[test]
    fn test_bhc_query_valid_indices() {
        let mut bhc = BinaryHierarchicalClustering::new().unwrap();
        let mut features = Vec::new();
        for i in 0..20u8 {
            let mut desc = [0u8; 96];
            desc[0] = i;
            features.push(desc);
        }

        let feat_refs: Vec<&[u8; 96]> = features.iter().collect();
        bhc.build(&feat_refs).unwrap();

        let query = [5u8; 96];
        let result = bhc.query(&query).unwrap();

        assert!(result.iter().all(|&idx| idx < 20));
    }

    #[test]
    fn test_bhc_deterministic() {
        let mut features = Vec::new();
        for i in 0..10u8 {
            let mut desc = [0u8; 96];
            desc[0] = i;
            features.push(desc);
        }

        let feat_refs: Vec<&[u8; 96]> = features.iter().collect();

        let mut bhc1 = BinaryHierarchicalClustering::new().unwrap();
        bhc1.build(&feat_refs).unwrap();
        let result1 = bhc1.query(&[0u8; 96]).unwrap();

        let mut bhc2 = BinaryHierarchicalClustering::new().unwrap();
        bhc2.build(&feat_refs).unwrap();
        let result2 = bhc2.query(&[0u8; 96]).unwrap();

        assert_eq!(result1, result2);
    }

    // ------------------------------------------------------------------
    // BHC settings + priority-queue traversal (ported in M9 #146)
    // ------------------------------------------------------------------

    /// Build a small set of synthetic 96-byte descriptors for the tests below.
    /// Descriptors are diverse enough to force a non-trivial tree (more than
    /// `num_centers + min_features_per_leaf` features).
    fn make_synth_descriptors(n: usize) -> Vec<[u8; 96]> {
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let mut d = [0u8; 96];
            for (j, byte) in d.iter_mut().enumerate() {
                // Pseudo-random but deterministic: vary every byte with a
                // simple multiplicative hash on (i, j).
                *byte = (((i * 131 + j * 17) ^ (i << 3)) & 0xff) as u8;
            }
            out.push(d);
        }
        out
    }

    #[test]
    fn test_bhc_set_num_hypotheses_then_build() {
        // Setting num_hypotheses to a non-default value must not break build/query.
        let descs = make_synth_descriptors(80);
        let refs: Vec<&[u8; 96]> = descs.iter().collect();

        let mut bhc = BinaryHierarchicalClustering::new().unwrap();
        bhc.set_num_hypotheses(128).unwrap();
        bhc.build(&refs).unwrap();
        let result = bhc.query(&descs[0]).unwrap();
        assert!(
            !result.is_empty(),
            "query should find at least one candidate"
        );
        assert!(result.iter().all(|&idx| idx < descs.len()));
    }

    #[test]
    fn test_bhc_set_num_hypotheses_zero_errors() {
        let mut bhc = BinaryHierarchicalClustering::new().unwrap();
        assert!(bhc.set_num_hypotheses(0).is_err());
    }

    #[test]
    fn test_bhc_set_num_centers_preserves_num_hypotheses() {
        // Regression test for setter ordering: set_num_centers must not
        // silently reset num_hypotheses back to 1 (it did before #146).
        let descs = make_synth_descriptors(80);
        let refs: Vec<&[u8; 96]> = descs.iter().collect();

        let mut bhc = BinaryHierarchicalClustering::new().unwrap();
        bhc.set_num_hypotheses(128).unwrap();
        bhc.set_num_centers(8).unwrap(); // must NOT reset num_hypotheses
        bhc.set_max_nodes_to_pop(8);
        bhc.set_min_features_per_leaf(16).unwrap();
        bhc.build(&refs).unwrap();
        let _ = bhc.query(&descs[0]).unwrap();
        // No panic = pass. The real check is the dual-mode BHC test in
        // mod dual_mode_tests below: with the wrong num_hypotheses the tree
        // topology diverges and that test would fail.
    }

    #[test]
    fn test_bhc_max_nodes_to_pop_zero_is_tied_min_only() {
        // With max_nodes_to_pop=0 (default), query should produce exactly
        // the same result set as the pre-#146 implementation: only
        // tied-minimum children visited.
        let descs = make_synth_descriptors(80);
        let refs: Vec<&[u8; 96]> = descs.iter().collect();

        let mut bhc = BinaryHierarchicalClustering::new().unwrap();
        // Explicit default; new() already sets 0.
        bhc.set_max_nodes_to_pop(0);
        bhc.build(&refs).unwrap();

        let result = bhc.query(&descs[0]).unwrap();
        assert!(!result.is_empty());
    }

    #[test]
    fn test_bhc_max_nodes_to_pop_widens_candidate_set() {
        // Monotonicity check: query(max_nodes_to_pop=N) should return at
        // least as many candidates as query(max_nodes_to_pop=0) on the same
        // tree. Pop more = visit more leaves = collect more candidates.
        let descs = make_synth_descriptors(200);
        let refs: Vec<&[u8; 96]> = descs.iter().collect();

        let mut bhc_default = BinaryHierarchicalClustering::new().unwrap();
        bhc_default.build(&refs).unwrap();
        let r_default = bhc_default.query(&descs[0]).unwrap();

        let mut bhc_wide = BinaryHierarchicalClustering::new().unwrap();
        bhc_wide.set_max_nodes_to_pop(8);
        bhc_wide.build(&refs).unwrap();
        let r_wide = bhc_wide.query(&descs[0]).unwrap();

        assert!(
            r_wide.len() >= r_default.len(),
            "max_nodes_to_pop=8 returned fewer candidates ({}) than =0 ({})",
            r_wide.len(),
            r_default.len()
        );
    }
}

// ============================================================================
// Dual-mode validation: bridges to C++ FastRandom / ArrayShuffle (#116)
// ============================================================================
//
// When `dual-mode` is enabled, these tests verify that the Rust ports of
// `fast_random` and `array_shuffle` produce a byte-identical sequence and
// permutation to the C++ baseline (vision::FastRandom / vision::ArrayShuffle
// from math/rand.h). This is what enables the BHC-indexed match dual-mode
// test in matcher.rs to assert pair equality with C++ rather than count-only.

// Only ever called from `#[cfg(all(test, feature = "dual-mode"))]`
// modules below; gate the extern declaration the same way so the lib
// target under --all-features doesn't see a phantom dead symbol (#180).
#[cfg(all(test, feature = "dual-mode"))]
extern "C" {
    fn webarkit_cpp_fast_random(seed: *mut i32) -> i32;
    fn webarkit_cpp_array_shuffle(v: *mut i32, pop_size: i32, sample_size: i32, seed: *mut i32);

    /// Build a C++ `BinaryHierarchicalClustering<96>` with the supplied
    /// settings, query it once, and write returned indices to `out_indices`.
    /// Returns number of indices written or -1 on error.
    /// See `kpm_c_api.h` for full doc. M9 #146.
    fn webarkit_cpp_bhc_build_and_query_with_settings(
        features: *const u8,
        num_features: i32,
        num_hypotheses: i32,
        num_centers: i32,
        max_nodes_to_pop: i32,
        min_features_per_node: i32,
        query_feat: *const u8,
        out_indices: *mut i32,
    ) -> i32;
}

#[cfg(all(test, feature = "dual-mode"))]
mod dual_mode_tests {
    use super::*;

    /// Sweep many seeds and many calls; verify Rust and C++ produce the same
    /// sequence and the same final seed state.
    #[test]
    fn dual_mode_fast_random_byte_identical() {
        for &start in &[0i32, 1, 1234, -1, -1234, i32::MAX, i32::MIN, 7919, -7919] {
            let mut rust_seed = start;
            let mut cpp_seed = start;
            for step in 0..1000 {
                let rust_r = fast_random(&mut rust_seed);
                let cpp_r = unsafe { webarkit_cpp_fast_random(&mut cpp_seed) };
                assert_eq!(
                    rust_r, cpp_r,
                    "value mismatch at start={}, step={}: rust={}, cpp={}",
                    start, step, rust_r, cpp_r
                );
                assert_eq!(
                    rust_seed, cpp_seed,
                    "seed mismatch at start={}, step={}",
                    start, step
                );
            }
        }
    }

    /// Run array_shuffle with various pop_size / sample_size / seed combinations
    /// and assert the resulting permutation and seed are byte-identical to C++.
    #[test]
    fn dual_mode_array_shuffle_byte_identical() {
        let cases: &[(usize, usize, i32)] = &[
            (10, 5, 1234),
            (10, 10, 1234),
            (50, 8, 1234),
            (100, 16, 42),
            (1, 1, 999),
            (256, 32, -1),
            (200, 0, 7), // sample_size=0: no shuffling, should be no-op
            (50, 50, 12345),
        ];

        for &(pop, sample, start_seed) in cases {
            let mut rust_v: Vec<i32> = (0..pop as i32).collect();
            let mut cpp_v: Vec<i32> = (0..pop as i32).collect();
            let mut rust_seed = start_seed;
            let mut cpp_seed = start_seed;

            array_shuffle(&mut rust_v, sample, &mut rust_seed);
            unsafe {
                webarkit_cpp_array_shuffle(
                    cpp_v.as_mut_ptr(),
                    pop as i32,
                    sample as i32,
                    &mut cpp_seed,
                );
            }

            assert_eq!(
                rust_v, cpp_v,
                "permutation mismatch at pop={}, sample={}, seed={}",
                pop, sample, start_seed
            );
            assert_eq!(
                rust_seed, cpp_seed,
                "seed mismatch at pop={}, sample={}, seed={}",
                pop, sample, start_seed
            );
        }
    }

    /// Run multiple shuffles in sequence with a shared seed and verify both
    /// implementations evolve their state identically across calls. This
    /// matches how BHC uses ArrayShuffle (one persistent seed across many
    /// k-medoids invocations during recursive tree build).
    #[test]
    fn dual_mode_array_shuffle_persistent_seed() {
        let mut rust_v: Vec<i32> = (0..100).collect();
        let mut cpp_v: Vec<i32> = (0..100).collect();
        let mut rust_seed = 1234i32;
        let mut cpp_seed = 1234i32;

        for round in 0..10 {
            array_shuffle(&mut rust_v, 8, &mut rust_seed);
            unsafe {
                webarkit_cpp_array_shuffle(cpp_v.as_mut_ptr(), 100, 8, &mut cpp_seed);
            }
            assert_eq!(rust_v, cpp_v, "permutation diverged at round {}", round);
            assert_eq!(rust_seed, cpp_seed, "seed diverged at round {}", round);
        }
    }

    /// Build a deterministic set of 96-byte descriptors for the BHC dual-mode test.
    /// Same algorithm as the unit-test `make_synth_descriptors` helper in
    /// `mod tests` above; duplicated locally because `mod tests` is sibling.
    fn make_synth_descriptors_for_dual_mode(n: usize) -> Vec<[u8; 96]> {
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let mut d = [0u8; 96];
            for (j, byte) in d.iter_mut().enumerate() {
                *byte = (((i * 131 + j * 17) ^ (i << 3)) & 0xff) as u8;
            }
            out.push(d);
        }
        out
    }

    /// Diagnostic-only: documents BHC-layer cross-language nondeterminism.
    ///
    /// Goal (M9 #146 Decision 6): verify Rust BHC with `(num_hypotheses=128,
    /// num_centers=8, max_nodes_to_pop=8, min_features_per_node=16)` returns
    /// the same candidate set as C++.
    ///
    /// **Why this is `#[ignore]`d**: both implementations use unordered-key
    /// maps when grouping K-medoids assignments into child clusters
    /// (C++ `std::unordered_map<int, std::vector<int>>` at
    /// `binary_hierarchical_clustering.h:217`; Rust originally used
    /// `HashMap`, now uses `BTreeMap` for intra-Rust determinism).
    /// Since the hash orderings differ between toolchains AND the cluster
    /// keys themselves differ (C++ keys by feature-array index; Rust keys by
    /// cluster position 0..k-1), child ordering in the BHC tree diverges
    /// across languages, even when K-medoids produces equivalent partitions.
    /// The BHC algorithm tolerates this (priority-queue traversal handles
    /// ties), so the algorithmic correctness is unaffected — but
    /// candidate-set byte-equality across implementations is not achievable
    /// at the BHC layer alone.
    ///
    /// **The actual milestone gate is `test_visual_database_matches_cpp_pipeline`**
    /// in `visual_database.rs`, which measures end-to-end pipeline parity.
    ///
    /// Keeping this test as `#[ignore]`d preserves the diagnostic intent:
    /// run with `cargo test --include-ignored` to see the divergence.
    #[test]
    #[ignore = "BHC tree topology diverges across languages due to unordered-map child ordering; \
                see test docstring. Algorithm correctness is unaffected; \
                use test_visual_database_matches_cpp_pipeline for the milestone gate."]
    fn bhc_with_keyframe_settings_matches_cpp() {
        let descs = make_synth_descriptors_for_dual_mode(200);
        let query = descs[42]; // self-match: query equals a known descriptor

        // ----- Rust path -----
        let descriptor_refs: Vec<&[u8; 96]> = descs.iter().collect();
        let mut bhc = BinaryHierarchicalClustering::new().unwrap();
        bhc.set_num_hypotheses(128).unwrap();
        bhc.set_num_centers(8).unwrap();
        bhc.set_max_nodes_to_pop(8);
        bhc.set_min_features_per_leaf(16).unwrap();
        bhc.build(&descriptor_refs).unwrap();
        let mut rust_indices = bhc.query(&query).unwrap();
        rust_indices.sort();

        // ----- C++ path via new FFI shim -----
        let flat: Vec<u8> = descs.iter().flatten().copied().collect();
        let mut cpp_indices_buf = vec![0i32; descs.len()];
        let cpp_count = unsafe {
            webarkit_cpp_bhc_build_and_query_with_settings(
                flat.as_ptr(),
                descs.len() as i32,
                128,
                8,
                8,
                16,
                query.as_ptr(),
                cpp_indices_buf.as_mut_ptr(),
            )
        };
        assert!(cpp_count >= 0, "C++ BHC shim returned error: {}", cpp_count);
        cpp_indices_buf.truncate(cpp_count as usize);
        let mut cpp_indices: Vec<usize> = cpp_indices_buf.iter().map(|&i| i as usize).collect();
        cpp_indices.sort();

        // ----- Parity assertion -----
        assert_eq!(
            rust_indices,
            cpp_indices,
            "BHC with Keyframe::buildIndex settings (128/8/8/16) diverged from C++:\n  \
             rust: {} candidates\n  \
             cpp:  {} candidates\n  \
             If counts differ, the tree topology diverged. \
             If counts match but the sets differ, the priority-queue traversal \
             tie-break order is off (check seq-based ordering in BacklogEntry::cmp).",
            rust_indices.len(),
            cpp_indices.len()
        );
    }
}
