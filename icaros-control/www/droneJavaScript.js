let toggleNavStatus = false;

let toggleNav = function() {
    let getSidebar = document.querySelector(".nav-sidebar");
    let getSidebarUl = document.querySelector(".nav-sidebar ul");
    let getSidebarTitle = document.querySelector(".nav-sidebar span");
    let getSidebarA = document.querySelectorAll(".nav-sidebar a");

    let mainBody = document.querySelector(".mainBody");

    if (toggleNavStatus === false){
        getSidebarUl.style.visibility = "visible";
        getSidebar.style.width = "300px";
        getSidebarTitle.style.opacity = "0.5";

        mainBody.style.padding = "60px 0px 0px 310px";

        let arrayLength = getSidebarA.length;
        for(var i = 0; i < arrayLength; i++){
            getSidebarA[i].style.opacity = "1";
        }

        toggleNavStatus = true;
    }
    else if (toggleNavStatus === true){

        getSidebar.style.width = "50px";
        getSidebarTitle.style.opacity = "0";

        mainBody.style.padding = "60px 0px 0px 60px";

        let arrayLength = getSidebarA.length;
        for(var i = 0; i < arrayLength; i++){
            getSidebarA[i].style.opacity = "0";
        }

        getSidebarUl.style.visibility = "hidden";
        toggleNavStatus = false;
    }
}

mapboxgl.accessToken = 'pk.eyJ1IjoiYWxiZXJ0b3VhIiwiYSI6ImNrNmRoejBocjA3N28zbG52cXc2eWo4MG8ifQ.SvZlpvjSEalfP7q37YIc4Q';

var mymap = window.mymap = L.map('map').setView([51.505, -0.09], 13);

L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token={accessToken}', {
    attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors, <a href="https://creativecommons.org/licenses/by-sa/2.0/">CC-BY-SA</a>, Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
    maxZoom: 18,
    id: 'mapbox/streets-v11',
    accessToken: 'your.mapbox.access.token'
}).addTo(mymap);

var marker = L.marker([51.5, -0.09]).addTo(mymap);

var circle = L.circle([51.508, -0.11], {
    color: 'red',
    fillColor: '#f03',
    fillOpacity: 0.5,
    radius: 500
}).addTo(mymap);
