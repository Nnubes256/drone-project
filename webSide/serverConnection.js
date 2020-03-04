//open web socket

let socket = new WebSocket("serverURL");

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

  speedTL = dataReceived.drone.motor_speeds.tl, tr, bl, br,
  speedTR = dataReceived.drone.motor_speeds.tr;
  speedBL = dataReceived.drone.motor_speeds.bl;
  speedBR = dataReceived.drone.motor_speeds.br;
  x = dataReceived.drone.orientation.x;
  y = dataReceived.drone.orientation.y;
  z = dataReceived.drone.orientation.z;
  w = dataReceived.drone.orientation.w;
  accelerationX = dataReceived.drone.acceleration.x;
  accelerationY = dataReceived.drone.acceleration.y;
  accelerationZ = dataReceived.drone.acceleration.z;
  var altitude = dataReceived.drone.gps.altitude;
  LatitudeLayer.textContent = altitude;
  var longitude = dataReceived.drone.gps.longitude;
  LongitudeLayer.textContent = longitude;
  AltitudeLayer.textContent = dataReceived.drone.gps.altitude;
  SpeedLayer.textContent = dataReceived.drone.gps.speed;
  let roll = dataReceived.gamepad.axis_state.roll; 
  PtichLayer.textContent = dataReceived.gamepad.axis_state.pitch;
  RawLayer.textContent = dataReceived.gamepad.axis_state.raw;
  ThrottleLayer.textContent = dataReceived.gamepad.axis_state.throttle;
  angleImage.style.rotate = roll;
  RollLayer.textContent = roll;

  if(axisThrottle === 1050)
    lowSpeedLED.style.backgroundColor = "red";
  else
    lowSpeedLED.style.backgroundColor = "white";

  if(time > 560)
    lowBatteryLED.style.backgroundColor = "red";
  else
    lowBatteryLED.style.backgroundColor = "white";

  if(axisPitch > 60 || axisRaw > 60 || axisRoll > 60)
    dangerousAngleLED.style.backgroundColor = "red";
  else
    dangerousAngleLED.style.backgroundColor = "white";
      
    function coordinateFeature(lng, lat) {
      return {
        center: [lng, lat],
        geometry: {
        type: 'Point',
        coordinates: [lng, lat]
        },
        place_name: 'Lat: ' + lat + ' Lng: ' + lng,
        place_type: ['coordinate'],
        properties: {},
        type: 'Feature'
      };
    }
    
    geocodes.push(coordinateFeature(23, 23));
    
    map.addControl(
      new MapboxGeocoder({
        accessToken: mapboxgl.accessToken,
        localGeocoder: coordinatesGeocoder,
        zoom: 4,
        placeholder: 'Try: -40, 170',
        mapboxgl: mapboxgl
      })
    );
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



var coordinatesGeocoder = function(Longitude, latitude) {
                            
  function coordinateFeature(lng, lat) {
      return {
          center: [lng, lat],
          geometry: {
          type: 'Point',
          coordinates: [lng, lat]
          },
          place_name: 'Lat: ' + lat + ' Lng: ' + lng,
          place_type: ['coordinate'],
          properties: {},
          type: 'Feature'
      };
  }

  var coord1 = Longitude;
  var coord2 = latitude;
  var geocodes = [];
  
  if (coord1 < -90 || coord1 > 90) {
      // must be lng, lat
      geocodes.push(coordinateFeature(coord1, coord2));
  }
  
  if (coord2 < -90 || coord2 > 90) {
      // must be lat, lng
      geocodes.push(coordinateFeature(coord2, coord1));
  }
  
  if (geocodes.length === 0) {
      // else could be either lng, lat or lat, lng
      geocodes.push(coordinateFeature(coord1, coord2));
      geocodes.push(coordinateFeature(coord2, coord1));
  }
  
  return geocodes;
};
