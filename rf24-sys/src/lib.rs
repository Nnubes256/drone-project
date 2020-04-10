#![feature(static_nobundle)]

#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(invalid_value)]
#![allow(deprecated)]

#[cfg(target_os = "linux")]
include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

#[cfg(not(target_os = "linux"))]
include!("bindings-dev.rs");
