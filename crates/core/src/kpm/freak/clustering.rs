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
use crate::{arlog_d, arlog_e, arlog_i};
use std::collections::HashMap;

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
    // SAFETY: Transmuting &[u8; 96] to &[u32; 24] is safe because:
    // 1. Byte arrays are correctly sized (96 bytes = 24 × 4 bytes)
    // 2. u32 alignment is guaranteed on all supported targets
    // 3. The transmute creates a properly-aligned view of the same data
    let a32 = unsafe { std::mem::transmute::<&[u8; 96], &[u32; 24]>(a) };
    let b32 = unsafe { std::mem::transmute::<&[u8; 96], &[u32; 24]>(b) };

    a32.iter()
        .zip(b32.iter())
        .map(|(x, y)| hamming_distance_32(*x, *y))
        .sum()
}

/// Lightweight linear congruential RNG for deterministic clustering.
struct SimpleRng {
    seed: u64,
}

impl SimpleRng {
    fn new(seed: u64) -> Self {
        Self { seed }
    }

    fn next(&mut self) -> u64 {
        self.seed = self.seed.wrapping_mul(1103515245).wrapping_add(12345);
        self.seed
    }

    fn next_usize(&mut self, min: usize, max: usize) -> usize {
        let range = (max - min) as u64;
        if range == 0 {
            return min;
        }
        min + ((self.next() % range) as usize)
    }
}

/// K-medoids clustering for binary features (96-byte FREAK descriptors).
/// C equivalent: BinarykMedoids<96>
pub struct KMedoids {
    k: usize,
    centers: Vec<usize>,
    assignment: Vec<usize>,
    num_hypotheses: usize,
    rand_seed: u64,
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
    pub fn set_rand_seed(&mut self, seed: u64) {
        self.rand_seed = seed;
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

        self.assignment.resize(features.len(), 0);

        let mut best_distortion = u64::MAX;
        let mut best_assignment = vec![0; features.len()];
        let mut best_centers = vec![0; self.k];

        for hyp in 0..self.num_hypotheses {
            let mut rng = SimpleRng::new(self.rand_seed.wrapping_add(hyp as u64));
            let mut candidate_indices: Vec<usize> = (0..features.len()).collect();

            for i in 0..self.k {
                let j = rng.next_usize(i, features.len());
                candidate_indices.swap(i, j);
            }
            let hypothesis_centers: Vec<usize> = candidate_indices[..self.k].to_vec();

            let mut hyp_assignment = vec![0; features.len()];
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
    /// Child nodes.
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
    min_features_per_leaf: usize,
    max_nodes_to_pop: usize,
    next_node_id: usize,
}

impl BinaryHierarchicalClustering {
    /// Creates a new empty binary hierarchical clustering tree.
    pub fn new() -> Result<Self, KpmError> {
        let mut kmedoids = KMedoids::new(8, 1)?;
        kmedoids.set_rand_seed(1234);

        Ok(Self {
            root: None,
            kmedoids,
            num_centers: 8,
            min_features_per_leaf: 16,
            max_nodes_to_pop: 0,
            next_node_id: 0,
        })
    }

    /// Sets the number of centers per split in the tree.
    pub fn set_num_centers(&mut self, k: usize) -> Result<(), KpmError> {
        if k == 0 {
            arlog_e!("BinaryHierarchicalClustering::set_num_centers: k must be > 0");
            return Err(KpmError::InvalidInput("k must be > 0".into()));
        }
        self.num_centers = k;
        self.kmedoids = KMedoids::new(k, 1)?;
        Ok(())
    }

    /// Sets the minimum number of features per leaf node.
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
    pub fn query(&self, query_feature: &[u8; 96]) -> Result<Vec<usize>, KpmError> {
        let root = self.root.as_ref().ok_or_else(|| {
            arlog_e!("BinaryHierarchicalClustering::query: tree not built");
            KpmError::InternalError("tree not built".into())
        })?;

        let mut result = Vec::new();
        self.query_recursive(root, query_feature, &mut result)?;

        arlog_d!(
            "BinaryHierarchicalClustering::query: found {} features",
            result.len()
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

        let mut clusters: HashMap<usize, Vec<usize>> = HashMap::new();

        for (feat_idx_in_subset, &cluster_assignment) in assignment.iter().enumerate() {
            let global_feat_idx = indices[feat_idx_in_subset];
            clusters
                .entry(cluster_assignment)
                .or_insert_with(Vec::new)
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

    fn query_recursive(
        &self,
        node: &BhcNode,
        query_feature: &[u8; 96],
        result: &mut Vec<usize>,
    ) -> Result<(), KpmError> {
        if node.is_leaf {
            result.extend_from_slice(&node.reverse_index);
            return Ok(());
        }

        // Find nearest child based on Hamming distance to center
        let mut nearest_idx = 0;
        let mut nearest_dist = u32::MAX;

        for (i, child) in node.children.iter().enumerate() {
            if let Some(center) = &child.center {
                let dist = hamming_distance_96(center, query_feature);
                if dist < nearest_dist {
                    nearest_dist = dist;
                    nearest_idx = i;
                }
            }
        }

        // Recurse on nearest child only
        self.query_recursive(&node.children[nearest_idx], query_feature, result)?;

        Ok(())
    }
}
#[cfg(test)]
mod tests {
    use super::*;

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
}
