// Obtain the video
var video_cont = document.getElementById('video');

// Create our decoder
var wsavc = new WSAvcPlayer.default({ useWorker:true });

// Add the decoder's video output to our webpage
document.getElementById('videoPlayer').appendChild(wsavc.AvcPlayer.canvas);

// Connect to the video stream
var uri = "ws://" + document.location.host + "/video";
wsavc.connect(uri);

// Events
wsavc.on('disconnected', () => console.log('Video websocket disconnected'));
wsavc.on('connected', () => console.log('Video websocket connected'));
wsavc.on('resized', (payload) => {
    console.log('resized', payload);
});
wsavc.on('stream_active', active => console.log('Stream is', active ? 'active' : 'offline'));
