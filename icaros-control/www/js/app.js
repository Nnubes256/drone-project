(function() {
    'use strict';

    let exampleSocket = new WebSocket("ws://127.0.0.1:3030/feed");

    exampleSocket.onopen = (event) => {
        document.getElementById("conn_status").textContent = "Connected!";
    }

    exampleSocket.onclose = (event) => {
        document.getElementById("conn_status").textContent = "Closed";
    }

    exampleSocket.onmessage = (event) => {
        let msg = JSON.parse(event.data);
        if (msg.x != null) {
            document.getElementById("x").textContent = `${msg.x}`;
        }
        if (msg.y != null) {
            document.getElementById("y").textContent = `${msg.y}`;
        }
        if (msg.z != null) {
            document.getElementById("z").textContent = `${msg.z}`;
        }
        if (msg.ground != null) {
            document.getElementById("time").textContent = `${JSON.stringify(msg.ground)}`;
        }
        if (msg.gamepad) {
            if (msg.gamepad.axis_state) {
                if (msg.gamepad.axis_state.roll != null) document.getElementById("gpad_roll").textContent = `${msg.gamepad.axis_state.roll}`;
                if (msg.gamepad.axis_state.pitch != null) document.getElementById("gpad_pitch").textContent = `${msg.gamepad.axis_state.pitch}`;
                if (msg.gamepad.axis_state.yaw != null) document.getElementById("gpad_yaw").textContent = `${msg.gamepad.axis_state.yaw}`;
                if (msg.gamepad.axis_state.throttle != null) document.getElementById("gpad_throttle").textContent = `${msg.gamepad.axis_state.throttle}`;
            }
            if (msg.drone.orientation.throttle_lock != null) document.getElementById("gpad_lock").textContent = `${JSON.stringify(msg.gamepad.throttle_lock)}`;
        }
        if (msg.drone) {
            if (msg.drone.motor_speeds) {
                if (msg.drone.motor_speeds.tl != null) document.getElementById("tl").textContent = `${msg.drone.motor_speeds.tl}`;
                if (msg.drone.motor_speeds.tr != null) document.getElementById("tr").textContent = `${msg.drone.motor_speeds.tr}`;
                if (msg.drone.motor_speeds.bl != null) document.getElementById("bl").textContent = `${msg.drone.motor_speeds.bl}`;
                if (msg.drone.motor_speeds.br != null) document.getElementById("br").textContent = `${msg.drone.motor_speeds.br}`;
            }
            if (msg.drone.orientation) {
                if (msg.drone.orientation.w != null) document.getElementById("qw").textContent = `${msg.drone.orientation.w}`;
                if (msg.drone.orientation.x != null) document.getElementById("qx").textContent = `${msg.drone.orientation.x}`;
                if (msg.drone.orientation.y != null) document.getElementById("qy").textContent = `${msg.drone.orientation.y}`;
                if (msg.drone.orientation.z != null) document.getElementById("qz").textContent = `${msg.drone.orientation.z}`;
            }
            if (msg.drone.vector) {
                if (msg.drone.vector.x != null) document.getElementById("ax").textContent = `${msg.drone.vector.x}`;
                if (msg.drone.vector.y != null) document.getElementById("ay").textContent = `${msg.drone.vector.y}`;
                if (msg.drone.vector.z != null) document.getElementById("az").textContent = `${msg.drone.vector.z}`;
            }
            if (msg.drone.gps) {
                if (msg.drone.gps.latitude != null) document.getElementById("lat").textContent = `${msg.drone.gps.latitude}`;
                if (msg.drone.gps.longitude != null) document.getElementById("lon").textContent = `${msg.drone.gps.longitude}`;
                if (msg.drone.gps.altitude != null) document.getElementById("alt").textContent = `${msg.drone.gps.altitude}`;
            }
        }
    }
}());