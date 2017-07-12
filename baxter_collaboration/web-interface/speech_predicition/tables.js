// Connecting to ROS
// ------------------------
var ros = new ROSLIB.Ros();
// Guess connection of the rosbridge websocket
function getRosBridgeHost() {
    if (window.location.protocol == 'file:') {
        return '192.168.1.3';
    } else {
        return window.location.hostname;
    }
}

var rosBridgePort = 9090;
// Create a connection to the rosbridge WebSocket server.
ros.connect('ws://' + getRosBridgeHost() + ':' + rosBridgePort);


var leftAruco  = new ROSLIB.Service({
    ros : ros,
    name: '/aruco_marker/service_left',
    messageType : 'baxter_collaboration_msgs/DoAction'
});

var rightAruco  = new ROSLIB.Service({
    ros : ros,
    name: '/action_provider/service_left',
    messageType : 'baxter_collaboration_msgs/DoAction'
});

var width  = 640,
    height = 480;

var left_svg = d3.select("#left-svg-container").select("svg")
                  //responsive SVG needs these 2 attributes and no width and height attr
                  .attr('preserveAspectRatio', 'xMinYMin meet')
                  .attr('viewBox', '0 0 ' + width + ' ' + height)
                  //class to make it responsive
                  .classed('svg-content-responsive', true);

var right_svg = d3.select("#right-svg-container").select("svg")
                  //responsive SVG needs these 2 attributes and no width and height attr
                  .attr('preserveAspectRatio', 'xMinYMin meet')
                  .attr('viewBox', '0 0 ' + width + ' ' + height)
                  //class to make it responsive
                  .classed('svg-content-responsive', true);

//Draw the Circle
var circle = left_svg.append("circle")
                         .attr("cx", 30)
                         .attr("cy", 30)
                         .attr("r", 20);
