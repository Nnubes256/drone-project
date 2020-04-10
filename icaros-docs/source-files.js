var N = null;var sourcesIndex = {};
sourcesIndex["icaros"] = {"name":"","dirs":[{"name":"comms","dirs":[{"name":"air","files":["dummy.rs","mod.rs","rf24.rs","simulated.rs","udp.rs"]},{"name":"controller","files":["mod.rs","scheduler.rs","serial.rs","simulated.rs"]}],"files":["application.rs","mod.rs"]},{"name":"hardware","files":["camera.rs","gps.rs","mod.rs","serialport.rs"]}],"files":["core.rs","esb.rs","main.rs"]};
sourcesIndex["icaros_base"] = {"name":"","dirs":[{"name":"comms","dirs":[{"name":"air","files":["scheduler.rs"]}],"files":["air.rs","common.rs","controller.rs","mod.rs"]},{"name":"utils","files":["crc16.rs","mod.rs","quartenion.rs","vector.rs"]}],"files":["core.rs","lib.rs"]};
sourcesIndex["icaros_control"] = {"name":"","dirs":[{"name":"comms","files":["mod.rs","rf24.rs","udp.rs"]},{"name":"web","files":["video.rs"]}],"files":["core.rs","gamepad.rs","main.rs","web.rs"]};
sourcesIndex["rf24"] = {"name":"","files":["lib.rs"]};
sourcesIndex["rf24_sys"] = {"name":"","files":["bindings-dev.rs","lib.rs"]};
createSourceSidebar();
