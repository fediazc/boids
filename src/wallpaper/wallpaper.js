/*
    Wallpaper engine setup
*/

Module['WEProperties'] = {
    boidColor: {r: 0, g: 255, b: 255},
    backgroundColor: {r: 30, g:30, b:30},
    numberOfBoids: 300,
    boidSpeed: 2,
    cohesion: 0.5,
    alignment: 0.5,
    separation: 0.75,
    visualRange: 45,
    drawVision: true
};

window.wallpaperPropertyListener = {
    applyUserProperties: function(properties) {
        if (properties.boidcolor) {
            var newBoidColor = properties.boidcolor.value.split(' ');
            newBoidColor = newBoidColor.map(function(c) {
                return Math.ceil(c * 255);
            });
            
            Module.WEProperties.boidColor.r = newBoidColor[0];
            Module.WEProperties.boidColor.g = newBoidColor[1];
            Module.WEProperties.boidColor.b = newBoidColor[2];
        }

        if (properties.backgroundcolor) {
            var newBGColor = properties.backgroundcolor.value.split(' ');
            newBGColor = newBGColor.map(function(c) {
                return Math.ceil(c * 255);
            });

            document.body.style.backgroundColor = 'rgb(' + newBGColor + ')';
            
            Module.WEProperties.backgroundColor.r = newBGColor[0];
            Module.WEProperties.backgroundColor.g = newBGColor[1];
            Module.WEProperties.backgroundColor.b = newBGColor[2];
        }

        if (properties.shownetwork) {
            Module.WEProperties.drawVision = properties.shownetwork.value;
        }

        if (properties.numberofboids) {
            Module.WEProperties.numberOfBoids = properties.numberofboids.value;
        }

        if (properties.boidspeed) {
            Module.WEProperties.boidSpeed = properties.boidspeed.value;
        }

        if (properties.cohesion) {
            Module.WEProperties.cohesion = properties.cohesion.value;
        }

        if (properties.boidalignment) {
            Module.WEProperties.alignment = properties.boidalignment.value;
        }

        if (properties.separation) {
            Module.WEProperties.separation = properties.separation.value;
        }

        if (properties.boidvisionrange) {
            Module.WEProperties.visualRange = properties.boidvisionrange.value;
        }
    }
};