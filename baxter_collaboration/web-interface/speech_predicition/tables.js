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

var svg = d3.select("svg");
