//! TubeCraft: a SolidCraft command plugin that turns sketch wireframes into bent, coped and
//! mitred round tube, one physical tube per run.
//!
//! The geometry and planning modules are plain Rust and are tested natively (`cargo test`).
//! Only the WebAssembly build links the SolidCraft plugin interface (`wit/plugin.wit`).

pub mod geom;
pub mod perp;
pub mod plan;
pub mod profiles;
pub mod sketch;
pub mod tube;

#[cfg(target_arch = "wasm32")]
mod component {
    wit_bindgen::generate!({ path: "wit", world: "command-plugin" });

    use craft::plugin::commands;
    use craft::plugin::types::{FieldFailure, PluginError};
    use exports::craft::plugin::entry::Guest;
    use serde_json::Value;

    struct Plugin;

    /// Runs SolidCraft commands for the entry.
    struct Host;

    impl crate::tube::Host for Host {
        type Err = PluginError;
        fn run(&mut self, command: &str, args: Value) -> Result<Value, PluginError> {
            let reply = commands::run(command, &args.to_string())?;
            serde_json::from_str(&reply).map_err(|e| invalid("reply", e.to_string()))
        }
        fn input_error(&self, field: &str, reason: String) -> PluginError {
            invalid(field, reason)
        }
    }

    impl Guest for Plugin {
        fn execute(entry: String, params: String) -> Result<String, PluginError> {
            let params: Value = serde_json::from_str(&params).map_err(|e| invalid("params", e.to_string()))?;
            if entry != "tube" {
                return Err(invalid("entry", format!("no entry `{entry}`")));
            }
            Ok(crate::tube::run(&mut Host, &params)?.to_string())
        }
    }

    fn invalid(field: &str, reason: String) -> PluginError {
        PluginError::InvalidInput(FieldFailure { field: field.into(), reason })
    }

    export!(Plugin);
}
