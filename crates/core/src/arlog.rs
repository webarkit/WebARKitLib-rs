/*
 *  arlog.rs
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
 *  Author(s): Walter Perdan @kalwalt https://github.com/kalwalt
 *
 */

//! ARToolKit-style logging on top of the [`log`] crate facade.
//!
//! Provides the familiar `arlog_d!`/`arlog_i!`/`arlog_w!`/`arlog_e!`/`arlog_rel!`/`arlog_perror!`
//! macros for code ported from the C++ WebARKitLib. Every call routes through
//! the standard `log` crate, so any backend (`env_logger`, `android_logger`,
//! `oslog`, `console_log`, `tracing`, …) works without change.
//!
//! # C ↔ Rust mapping
//!
//! | C macro                   | Rust macro               |
//! |---------------------------|--------------------------|
//! | `ARLOGd("x=%d", x);`      | `arlog_d!("x={}", x);`   |
//! | `ARLOGi("x=%d", x);`      | `arlog_i!("x={}", x);`   |
//! | `ARLOGw("x=%d", x);`      | `arlog_w!("x={}", x);`   |
//! | `ARLOGe("x=%d", x);`      | `arlog_e!("x={}", x);`   |
//! | `ARLOG("banner\n");`      | `arlog_rel!("banner");`  |
//! | `ARLOGperror("open");`    | `arlog_perror!("open");` |
//!
//! Drop the trailing `\n` — the macros emit newlines automatically through
//! the backend formatter.
//!
//! # Setup
//!
//! The `arlog_*!` macros are **always available** — they compile and run
//! without any extra feature. However, log records are only *displayed* when
//! a logger backend has been installed in the process; otherwise output is
//! silently dropped (this is standard `log` crate behavior).
//!
//! ## Option 1 — bundled initializer (`log-helpers` feature)
//!
//! With the `log-helpers` feature enabled, this crate ships a ready-made
//! initializer that reproduces the C ARToolKit output format
//! (`env_logger` on native, `console_log` on `wasm32`):
//!
//! ```no_run
//! # #[cfg(all(feature = "log-helpers", not(target_arch = "wasm32")))]
//! webarkitlib_rs::arlog::ar_log_init_default();
//! ```
//!
//! Produces output like `[info] hello`, `[warning] low mem`, `[error] bad fd`.
//!
//! For richer diagnostic output, use the verbose variant
//! [`ar_log_init_default_verbose`] (or [`ar_log_init_wasm_verbose`] on
//! `wasm32`):
//!
//! ```no_run
//! # #[cfg(all(feature = "log-helpers", not(target_arch = "wasm32")))]
//! webarkitlib_rs::arlog::ar_log_init_default_verbose();
//! ```
//!
//! Verbose output uses a single bracketed header:
//!
//! ```text
//! [info - 2026-04-21T14:23:45Z - webarkitlib_rs::marker] hello
//! [warning - 2026-04-21T14:23:45Z - webarkitlib_rs::ar2] low mem
//! ```
//!
//! Filtering is unchanged — `RUST_LOG` controls which records pass; the
//! verbose flag only changes how passing records are formatted.
//! [`arlog_rel!`] messages stay prefix-free in both modes.
//!
//! Examples that call `ar_log_init_default()` / `ar_log_init_wasm()` (or
//! their `_verbose` siblings) should
//! declare the dependency in `Cargo.toml` so `cargo run --example …` and
//! rust-analyzer pick the feature up automatically:
//!
//! ```toml
//! [[example]]
//! name = "load_nft"
//! required-features = ["log-helpers"]
//! ```
//!
//! See `examples/load_nft.rs` for the canonical usage pattern.
//!
//! ## Option 2 — bring your own backend
//!
//! If `log-helpers` is **off**, the `arlog_*!` macros still compile and emit
//! `log::Record`s. Install any `log`-compatible backend yourself
//! (`env_logger`, `tracing-subscriber`, `android_logger`, `oslog`, …) and
//! records will flow through it unchanged.
//!
//! # `RelInfo` semantics
//!
//! `arlog_rel!` routes through `target: "arlog::rel"` at `Level::Info`. To
//! reproduce the C behavior "only `RelInfo` passes", users set
//! `RUST_LOG=off,arlog::rel=info`. The bundled `ar_log_init_default()`
//! formatter emits `arlog::rel` messages without the `[level]` prefix.

/// Logging severity. Mirrors the C `AR_LOG_LEVEL_*` constants.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(i32)]
pub enum ArLogLevel {
    /// Debug level — per-frame rejections and trace detail.
    Debug = 0,
    /// Info level — one-shot informational events.
    Info = 1,
    /// Warn level — recoverable anomalies.
    Warn = 2,
    /// Error level — misconfiguration / wiring errors.
    Error = 3,
    /// Release-info: unconditional informational output with no `[level]` prefix.
    /// Routed through target `"arlog::rel"`.
    RelInfo = 4,
}

/// Matches the C `AR_LOG_LEVEL_DEFAULT`.
pub const AR_LOG_LEVEL_DEFAULT: ArLogLevel = ArLogLevel::Info;

impl From<ArLogLevel> for log::LevelFilter {
    fn from(v: ArLogLevel) -> Self {
        match v {
            ArLogLevel::Debug => log::LevelFilter::Debug,
            ArLogLevel::Info => log::LevelFilter::Info,
            ArLogLevel::Warn => log::LevelFilter::Warn,
            ArLogLevel::Error => log::LevelFilter::Error,
            // No exact equivalent in `log::LevelFilter`; `RelInfo` is a marker,
            // not a filter threshold. Map to Info so messages are not silenced.
            ArLogLevel::RelInfo => log::LevelFilter::Info,
        }
    }
}

impl From<log::LevelFilter> for ArLogLevel {
    fn from(v: log::LevelFilter) -> Self {
        match v {
            log::LevelFilter::Off | log::LevelFilter::Error => ArLogLevel::Error,
            log::LevelFilter::Warn => ArLogLevel::Warn,
            log::LevelFilter::Info => ArLogLevel::Info,
            log::LevelFilter::Debug | log::LevelFilter::Trace => ArLogLevel::Debug,
        }
    }
}

/// Sets the global minimum log level. Equivalent to assigning `arLogLevel` in C.
pub fn set_ar_log_level(level: ArLogLevel) {
    log::set_max_level(level.into());
}

/// Returns the current global minimum log level.
pub fn ar_log_level() -> ArLogLevel {
    log::max_level().into()
}

/// Target string used by [`arlog_rel!`] so backends can distinguish
/// release-info messages from plain `info` records.
pub const AR_LOG_REL_TARGET: &str = "arlog::rel";

#[macro_export]
/// Log a debug-level message (per-frame trace detail).
macro_rules! arlog_d {
    ($($arg:tt)*) => { ::log::debug!($($arg)*); };
}

#[macro_export]
/// Log an info-level message (one-shot events).
macro_rules! arlog_i {
    ($($arg:tt)*) => { ::log::info!($($arg)*); };
}

#[macro_export]
/// Log a warning-level message (recoverable anomalies).
macro_rules! arlog_w {
    ($($arg:tt)*) => { ::log::warn!($($arg)*); };
}

#[macro_export]
/// Log an error-level message (misconfiguration / wiring errors).
macro_rules! arlog_e {
    ($($arg:tt)*) => { ::log::error!($($arg)*); };
}

/// Release-info: informational output without a level prefix. Uses
/// `target: "arlog::rel"` so backends can format it without the `[info]` tag.
#[macro_export]
macro_rules! arlog_rel {
    ($($arg:tt)*) => {
        ::log::log!(target: $crate::arlog::AR_LOG_REL_TARGET, ::log::Level::Info, $($arg)*);
    };
}

/// C `ARLOGperror` equivalent: logs `std::io::Error::last_os_error()` at error
/// level, optionally prefixed by a caller-supplied string.
///
/// ```no_run
/// use webarkitlib_rs::arlog_perror;
/// arlog_perror!();            // → "[error] No such file or directory (os error 2)"
/// arlog_perror!("open");      // → "[error] open: No such file or directory (os error 2)"
/// ```
#[macro_export]
macro_rules! arlog_perror {
    () => {
        ::log::error!("{}", ::std::io::Error::last_os_error());
    };
    ($prefix:expr) => {
        ::log::error!("{}: {}", $prefix, ::std::io::Error::last_os_error());
    };
}

// --- Init helpers ---------------------------------------------------------

/// Maps a [`log::Level`] to the lower-case ARToolKit C label
/// (`error` / `warning` / `info` / `debug`). `Trace` collapses to `debug`.
#[cfg(not(target_arch = "wasm32"))]
#[allow(dead_code)]
fn level_label(level: log::Level) -> &'static str {
    match level {
        log::Level::Error => "error",
        log::Level::Warn => "warning",
        log::Level::Info => "info",
        log::Level::Debug | log::Level::Trace => "debug",
    }
}

/// Pure formatter used by the desktop init helpers. Extracted from the
/// `env_logger` closure so it can be unit-tested without installing a
/// global logger. `timestamp` is taken as a `&str` so tests can pass a
/// fixed value while production code passes `env_logger`'s timestamp.
#[cfg(not(target_arch = "wasm32"))]
#[allow(dead_code)]
fn format_line(
    verbose: bool,
    level: log::Level,
    target: &str,
    module: Option<&str>,
    timestamp: &str,
    args: &std::fmt::Arguments<'_>,
) -> String {
    // Release-info channel is always prefix-free, in both modes —
    // it's intended for clean banner / version output.
    if target == AR_LOG_REL_TARGET {
        return format!("{}", args);
    }
    let label = level_label(level);
    if verbose {
        let module = module.unwrap_or("?");
        format!("[{} - {} - {}] {}", label, timestamp, module, args)
    } else {
        format!("[{}] {}", label, args)
    }
}

/// Shared init for desktop env_logger backends. `verbose` toggles between the
/// terse C-style format (`[info] msg`) and the verbose unified format
/// (`[info - 2026-04-21T14:23:45Z - webarkitlib_rs::marker] msg`).
#[cfg(all(feature = "log-helpers", not(target_arch = "wasm32")))]
fn init_env_logger(verbose: bool) {
    use std::io::Write;
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format(move |buf, record| {
            let ts = buf.timestamp().to_string();
            let line = format_line(
                verbose,
                record.level(),
                record.target(),
                record.module_path(),
                &ts,
                record.args(),
            );
            writeln!(buf, "{}", line)
        })
        .init();
}

/// Installs [`env_logger`] with a formatter that reproduces ARToolKit's C
/// output format (`[info] msg`, `[warning] msg`, `[error] msg`, `[debug] msg`,
/// and unprefixed release-info). Honors `RUST_LOG` if set, defaults to `info`.
///
/// Call once in your binary's `main()` — never from library code.
///
/// ```no_run
/// webarkitlib_rs::arlog::ar_log_init_default();
/// // ... rest of your application
/// ```
#[cfg(all(feature = "log-helpers", not(target_arch = "wasm32")))]
pub fn ar_log_init_default() {
    init_env_logger(false);
}

/// Verbose variant of [`ar_log_init_default`]. Uses a single bracketed
/// header containing level, ISO-8601 UTC timestamp, and the originating
/// module path:
///
/// ```text
/// [info - 2026-04-21T14:23:45Z - webarkitlib_rs::marker] hello
/// ```
///
/// Release-info messages (via [`arlog_rel!`]) stay prefix-free even in
/// verbose mode, so they remain suitable for banners.
///
/// Filtering is unchanged from the terse variant — `RUST_LOG` still
/// controls which records pass; this only changes how they are formatted.
#[cfg(all(feature = "log-helpers", not(target_arch = "wasm32")))]
pub fn ar_log_init_default_verbose() {
    init_env_logger(true);
}

/// Installs [`console_log`] so `arlog_*!` macros appear in the browser DevTools
/// console. Call once from your WASM entry point.
#[cfg(all(feature = "log-helpers", target_arch = "wasm32"))]
pub fn ar_log_init_wasm() {
    console_log::init_with_level(log::Level::Info).expect("console_log init failed");
    log::set_max_level(log::LevelFilter::Info);
}

/// Verbose WASM variant: initializes [`console_log`] at `Debug` level so
/// `arlog_d!` records also reach the DevTools console.
///
/// Note: browser DevTools renders its own timestamp and source-location
/// columns for each console entry, so the verbose format used by the
/// desktop init helper is unnecessary here — DevTools already shows that
/// metadata natively.
#[cfg(all(feature = "log-helpers", target_arch = "wasm32"))]
pub fn ar_log_init_wasm_verbose() {
    console_log::init_with_level(log::Level::Debug).expect("console_log init failed");
    log::set_max_level(log::LevelFilter::Debug);
}

// --- Tests ----------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::{Mutex, OnceLock};
    use std::thread::ThreadId;

    type Captured = (log::Level, String, String);

    static CAPTURES: OnceLock<Mutex<Vec<Captured>>> = OnceLock::new();
    static TEST_LOCK: OnceLock<Mutex<()>> = OnceLock::new();
    static INIT: OnceLock<()> = OnceLock::new();

    // Identifies the thread currently running an arlog test. `cargo test`
    // runs tests in parallel on multiple threads and shares the global log
    // dispatcher across all of them, so without this filter logs emitted by
    // OTHER tests on other threads would leak into our capture buffer and
    // break our exact-count assertions.
    static CAPTURING_THREAD: Mutex<Option<ThreadId>> = Mutex::new(None);

    fn captures() -> &'static Mutex<Vec<Captured>> {
        CAPTURES.get_or_init(|| Mutex::new(Vec::new()))
    }

    fn test_lock() -> std::sync::MutexGuard<'static, ()> {
        TEST_LOCK
            .get_or_init(|| Mutex::new(()))
            .lock()
            .unwrap_or_else(|p| p.into_inner())
    }

    struct CaptureLogger;
    impl log::Log for CaptureLogger {
        fn enabled(&self, _m: &log::Metadata) -> bool {
            true
        }
        fn log(&self, record: &log::Record) {
            // Only capture logs from the thread currently running an arlog
            // test. Parallel tests on other threads still emit through the
            // global dispatcher, but their records are dropped here.
            let active = *CAPTURING_THREAD.lock().unwrap();
            if active != Some(std::thread::current().id()) {
                return;
            }
            captures().lock().unwrap().push((
                record.level(),
                record.target().to_string(),
                record.args().to_string(),
            ));
        }
        fn flush(&self) {}
    }

    static LOGGER: CaptureLogger = CaptureLogger;

    fn init_capture() {
        INIT.get_or_init(|| {
            let _ = log::set_logger(&LOGGER);
            log::set_max_level(log::LevelFilter::Trace);
        });
        // Mark THIS thread as the active capture target. test_lock() ensures
        // only one arlog test runs at a time, so this is safe to overwrite.
        *CAPTURING_THREAD.lock().unwrap() = Some(std::thread::current().id());
    }

    fn drain() -> Vec<Captured> {
        std::mem::take(&mut *captures().lock().unwrap())
    }

    #[test]
    fn level_roundtrip_debug() {
        let _g = test_lock();
        init_capture();
        set_ar_log_level(ArLogLevel::Debug);
        assert_eq!(ar_log_level(), ArLogLevel::Debug);
    }

    #[test]
    fn level_roundtrip_info() {
        let _g = test_lock();
        init_capture();
        set_ar_log_level(ArLogLevel::Info);
        assert_eq!(ar_log_level(), ArLogLevel::Info);
    }

    #[test]
    fn level_roundtrip_warn() {
        let _g = test_lock();
        init_capture();
        set_ar_log_level(ArLogLevel::Warn);
        assert_eq!(ar_log_level(), ArLogLevel::Warn);
    }

    #[test]
    fn level_roundtrip_error() {
        let _g = test_lock();
        init_capture();
        set_ar_log_level(ArLogLevel::Error);
        assert_eq!(ar_log_level(), ArLogLevel::Error);
    }

    #[test]
    fn level_relinfo_is_lossy() {
        let _g = test_lock();
        init_capture();
        set_ar_log_level(ArLogLevel::RelInfo);
        assert_eq!(ar_log_level(), ArLogLevel::Info);
    }

    #[test]
    fn level_filter_off_maps_to_error() {
        let _g = test_lock();
        init_capture();
        log::set_max_level(log::LevelFilter::Off);
        assert_eq!(ar_log_level(), ArLogLevel::Error);
    }

    #[test]
    fn level_filter_trace_maps_to_debug() {
        let _g = test_lock();
        init_capture();
        log::set_max_level(log::LevelFilter::Trace);
        assert_eq!(ar_log_level(), ArLogLevel::Debug);
    }

    #[test]
    fn arlog_i_emits_info_record() {
        let _g = test_lock();
        init_capture();
        log::set_max_level(log::LevelFilter::Trace);
        drain();
        arlog_i!("hello {}", 42);
        let records = drain();
        assert_eq!(records.len(), 1);
        let (lvl, _target, msg) = &records[0];
        assert_eq!(*lvl, log::Level::Info);
        assert_eq!(msg, "hello 42");
    }

    #[test]
    fn arlog_rel_uses_distinct_target() {
        let _g = test_lock();
        init_capture();
        log::set_max_level(log::LevelFilter::Trace);
        drain();
        arlog_rel!("banner v{}", "0.3.3");
        let records = drain();
        assert_eq!(records.len(), 1);
        let (lvl, target, msg) = &records[0];
        assert_eq!(*lvl, log::Level::Info);
        assert_eq!(target, AR_LOG_REL_TARGET);
        assert_eq!(msg, "banner v0.3.3");
    }

    #[test]
    fn arlog_perror_no_prefix() {
        let _g = test_lock();
        init_capture();
        log::set_max_level(log::LevelFilter::Error);
        drain();
        arlog_perror!();
        let records = drain();
        assert_eq!(records.len(), 1);
        let (lvl, _t, msg) = &records[0];
        assert_eq!(*lvl, log::Level::Error);
        assert!(!msg.is_empty());
        assert!(!msg.starts_with(": "), "unexpected prefix: {msg}");
    }

    #[test]
    fn arlog_perror_with_prefix() {
        let _g = test_lock();
        init_capture();
        log::set_max_level(log::LevelFilter::Error);
        drain();
        arlog_perror!("my_op");
        let records = drain();
        assert_eq!(records.len(), 1);
        let (lvl, _t, msg) = &records[0];
        assert_eq!(*lvl, log::Level::Error);
        assert!(msg.starts_with("my_op: "), "got: {msg}");
        assert!(msg.len() > "my_op: ".len(), "missing os error text: {msg}");
    }

    static EVAL_COUNT: AtomicUsize = AtomicUsize::new(0);
    fn side_effect() -> &'static str {
        EVAL_COUNT.fetch_add(1, Ordering::SeqCst);
        "x"
    }

    #[test]
    fn filtered_debug_arg_is_not_evaluated() {
        let _g = test_lock();
        init_capture();
        set_ar_log_level(ArLogLevel::Info);
        EVAL_COUNT.store(0, Ordering::SeqCst);
        arlog_d!("{}", side_effect());
        assert_eq!(
            EVAL_COUNT.load(Ordering::SeqCst),
            0,
            "arlog_d! must not evaluate args when filtered"
        );
    }

    // --- format_line tests (verbose mode wiring) ---

    const FAKE_TS: &str = "2026-04-21T14:23:45Z";

    #[test]
    fn format_line_terse_info() {
        let s = format_line(
            false,
            log::Level::Info,
            "webarkitlib_rs::marker",
            Some("webarkitlib_rs::marker"),
            FAKE_TS,
            &format_args!("hello {}", 42),
        );
        assert_eq!(s, "[info] hello 42");
    }

    #[test]
    fn format_line_terse_warn_uses_warning_label() {
        let s = format_line(
            false,
            log::Level::Warn,
            "x",
            Some("x"),
            FAKE_TS,
            &format_args!("low mem"),
        );
        assert_eq!(s, "[warning] low mem");
    }

    #[test]
    fn format_line_verbose_includes_ts_and_module() {
        let s = format_line(
            true,
            log::Level::Info,
            "webarkitlib_rs::marker",
            Some("webarkitlib_rs::marker"),
            FAKE_TS,
            &format_args!("hello"),
        );
        assert_eq!(
            s,
            "[info - 2026-04-21T14:23:45Z - webarkitlib_rs::marker] hello"
        );
    }

    #[test]
    fn format_line_verbose_falls_back_to_question_mark_module() {
        let s = format_line(
            true,
            log::Level::Error,
            "x",
            None,
            FAKE_TS,
            &format_args!("bad fd"),
        );
        assert_eq!(s, "[error - 2026-04-21T14:23:45Z - ?] bad fd");
    }

    #[test]
    fn format_line_rel_channel_is_prefix_free_in_terse_mode() {
        let s = format_line(
            false,
            log::Level::Info,
            AR_LOG_REL_TARGET,
            Some("webarkitlib_rs::version"),
            FAKE_TS,
            &format_args!("WebARKitLib-rs v0.3.3"),
        );
        assert_eq!(s, "WebARKitLib-rs v0.3.3");
    }

    #[test]
    fn format_line_rel_channel_is_prefix_free_in_verbose_mode() {
        let s = format_line(
            true,
            log::Level::Info,
            AR_LOG_REL_TARGET,
            Some("webarkitlib_rs::version"),
            FAKE_TS,
            &format_args!("WebARKitLib-rs v0.3.3"),
        );
        assert_eq!(s, "WebARKitLib-rs v0.3.3");
    }

    #[test]
    fn format_line_trace_collapses_to_debug_label() {
        let s = format_line(
            false,
            log::Level::Trace,
            "x",
            Some("x"),
            FAKE_TS,
            &format_args!("trace msg"),
        );
        assert_eq!(s, "[debug] trace msg");
    }

    #[test]
    fn unfiltered_debug_arg_is_evaluated() {
        let _g = test_lock();
        init_capture();
        set_ar_log_level(ArLogLevel::Debug);
        drain();
        EVAL_COUNT.store(0, Ordering::SeqCst);
        arlog_d!("{}", side_effect());
        assert_eq!(EVAL_COUNT.load(Ordering::SeqCst), 1);
        let records = drain();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].0, log::Level::Debug);
        assert_eq!(records[0].2, "x");
    }
}
