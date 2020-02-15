//open web socket

let socket = new WebSocket("cameraURL");

socket.onopen = function(e) {
    let noVideoMessage = document.querySelector(".noMessageStyle");
    noVideoMessage.style.visivility = "hidden";
};

socket.onmessage = function(event) {
    var img = Image.parse(event.data);
    document.getElementById('videoCameraID').img("img");
};


socket.onerror = function(error) {
    let noVideoMessage = document.querySelector(".noMessageStyle");
    noVideoMessage.style.visivility = "visible";
};