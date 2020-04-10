//open web socket

let socket = new WebSocket("ws://" + window.location.host + "/feed");

let lowBatteryLED = document.getElementById("lowBatteryLED");

setTimeout(function(){ alert(lowBatteryLED.style.backgroundColor = "red"); }, 560000);

socket.onopen = function(e) {
};

socket.onmessage = function(event) {
  dataReceived = JSON.parse(event.data);

  let lowSpeedLED = document.getElementById("lowSpeedLED");
  let dangerousAngleLED = document.getElementById("dangerousAngleLED");
  let SpeedLayer = document.getElementById("Speed");
  let PtichLayer = document.getElementById("Pitch");
  let RawLayer = document.getElementById("Raw");
  let RollLayer = document.getElementById("Roll");
  let ThrottleLayer = document.getElementById("Throttle");
  let LatitudeLayer = document.getElementById("Latitude");
  let LongitudeLayer = document.getElementById("Longitude");
  let AltitudeLayer = document.getElementById("Altitude");
  let angleImage = document.getElementById("angleImage");

  speedTL = dataReceived.drone.motor_speeds.tl;
  speedTR = dataReceived.drone.motor_speeds.tr;
  speedBL = dataReceived.drone.motor_speeds.bl;
  speedBR = dataReceived.drone.motor_speeds.br;
  x = dataReceived.drone.orientation.x;
  y = dataReceived.drone.orientation.y;
  z = dataReceived.drone.orientation.z;
  w = dataReceived.drone.orientation.w;
  accelerationX = dataReceived.drone.vector.x;
  accelerationY = dataReceived.drone.vector.y;
  accelerationZ = dataReceived.drone.vector.z;
  var latitude = dataReceived.drone.gps.latitude;
  LatitudeLayer.textContent = latitude;
  var longitude = dataReceived.drone.gps.longitude;
  LongitudeLayer.textContent = longitude;
  AltitudeLayer.textContent = dataReceived.drone.gps.altitude;
  let roll = dataReceived.gamepad.axis_state.roll;
  PtichLayer.textContent = dataReceived.gamepad.axis_state.pitch;
  RawLayer.textContent = dataReceived.gamepad.axis_state.yaw;
  ThrottleLayer.textContent = dataReceived.gamepad.axis_state.throttle;
  SpeedLayer.textContent = dataReceived.drone.gps.speed;
  angleImage.style.rotate = roll;
  RollLayer.textContent = roll;

  /*if(axisThrottle === 1050)
    lowSpeedLED.style.backgroundColor = "red";
  else
    lowSpeedLED.style.backgroundColor = "white";*/

  /*if(time > 560)
    lowBatteryLED.style.backgroundColor = "red";
  else
    lowBatteryLED.style.backgroundColor = "white";*/

  /*if(axisPitch > 60 || axisRaw > 60 || axisRoll > 60)
    dangerousAngleLED.style.backgroundColor = "red";
  else
    dangerousAngleLED.style.backgroundColor = "white";*/

  if (latitude != null && longitude != null)
    window.mymap.panTo([latitude, longitude]);
  else {
      // Do something
  }
};

socket.onclose = function(event) {
  if (event.wasClean) {
    alert(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
  } else {
    // e.g. server process killed or network down
    // event.code is usually 1006 in this case
    alert('[close] Connection died');
  }
};

socket.onerror = function(error) {
  alert(`[error] ${error.message}`);
};
