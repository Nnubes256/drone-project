//open web socket

let socket = new WebSocket("serverURL");

socket.onopen = function(e) {
  
};

socket.onmessage = function(event) {
  dataReceived = JSON.parse(event.data);

  let lowSpeedLED = document.getElementById("lowSpeedLED");
  let lowBatteryLED = document.getElementById("lowBatteryLED");
  let dangerousAngleLED = document.getElementById("dangerousAngleLED");
  let speedStats = document.getElementById("speedStats");
  let orientationStats = document.getElementById("orientationStats");
  let coordenatesStats = document.getElementById("coordenatesStats");
  let objectDetection = document.getElementById("objectDetection");
  let angleImage = document.getElementById("angleImage");

  if(dataReceived.lowSpeed === true)
    lowSpeedLED.style.backgroundColor = "red";
  else
    lowSpeedLED.style.backgroundColor = "white";

  if(dataReceived.lowBattery === true)
    lowBatteryLED.style.backgroundColor = "red";
  else
    lowBatteryLED.style.backgroundColor = "white";

  if(dataReceived.dangerousAngle === true)
    dangerousAngleLED.style.backgroundColor = "red";
  else
    dangerousAngleLED.style.backgroundColor = "white";

  speedStats.textContent = dataReceived.speedStats;
  orientationStats.textContent = dataReceived.orientationStats;
  coordenatesStats.textContent = dataReceived.coordenatesStats;
  objectDetection.textContent = dataReceived.objectDetection;
  angleImage.style.transform = rotate(dataReceived.angle);
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