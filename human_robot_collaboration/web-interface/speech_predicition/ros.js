// Connecting to ROS
// -----------------
var beginning = Date.now();
var ros = new ROSLIB.Ros();

// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error) {
  document.getElementById('connecting').style.display = 'none';
  document.getElementById('connected').style.display = 'none';
  document.getElementById('closed').style.display = 'none';
  document.getElementById('error').style.display = 'block';
  document.getElementById('troubleshooting').style.display = 'inline-block';
  console.log(error);
});

// Find out exactly when we made a connection.
ros.on('connection', function() {
  console.log('Connection made!');
  document.getElementById('connecting').style.display = 'none';
  document.getElementById('error').style.display = 'none';
  document.getElementById('closed').style.display = 'none';
  document.getElementById('connected').style.display = 'block';
});

ros.on('close', function() {
  console.log('Connection closed.');
  document.getElementById('connecting').style.display = 'none';
  document.getElementById('connected').style.display = 'none';
  document.getElementById('closed').style.display = 'inline-block';
  document.getElementById('error').style.display = 'inline-block';
});

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

// First, we create a Topic object with details of the topic's name and message type.
var logTopic = new ROSLIB.Topic({
  ros : ros,
  name : '/web_interface/log',
  messageType : 'std_msgs/String'
});

// First, we create a Topic object with details of the topic's name and message type.
var elemPressed = new ROSLIB.Topic({
  ros : ros,
  name : '/web_interface',
  messageType : 'std_msgs/String'
});

// Topic for error passing to the left arm
var errorPressedL = new ROSLIB.Topic({
  ros : ros,
  name : '/robot/digital_io/left_lower_button/state',
  messageType : 'baxter_core_msgs/DigitalIOState'
});

// Topic for error passing to the right arm
var errorPressedR = new ROSLIB.Topic({
  ros : ros,
  name : '/robot/digital_io/right_lower_button/state',
  messageType : 'baxter_core_msgs/DigitalIOState'
});

var leftAruco  = new ROSLIB.Topic({
    ros : ros,
    name: '/aruco_marker_publisher/markers',
    messageType : 'aruco_msgs/MarkerArray'
});

var rightAruco  = new ROSLIB.Topic({
    ros : ros,
    name: '/hsv_detector/objects',
    messageType : 'human_robot_collaboration_msgs/ObjectsArray'
});

var rightArmInfo = new ROSLIB.Topic({
    ros : ros,
    name: '/action_provider/right/state',
    messageType : 'human_robot_collaboration_msgs/ArmState'
});

var leftArmInfo = new ROSLIB.Topic({
    ros : ros,
    name: '/action_provider/left/state',
    messageType : 'human_robot_collaboration_msgs/ArmState'
});

// Service Client to interface with the left arm
var leftArmService  = new ROSLIB.Service({
  ros : ros,
  name: '/action_provider/service_left',
  messageType : 'human_robot_collaboration_msgs/DoAction'
});

// Service Client to interface with the right arm
var rightArmService = new ROSLIB.Service({
  ros : ros,
  name: '/action_provider/service_right',
  messageType : 'human_robot_collaboration_msgs/DoAction'
});

var startExperiment = new ROSLIB.Service({
    ros : ros,
    name: '/rosbag/start',
    messageType :'std_srvs/Empty'
});

var stopExperiment = new ROSLIB.Service({
    ros : ros,
    name: '/rosbag/stop',
    messageType :'std_srvs/Empty'
});

// Add a callback for any element on the page
function callback(e) {
    var e = window.e || e;

    // Access the parameter server to get IDs of objs
    // corresponding to buttons presses

    //These are defines in speech_prediction.launch
    var left_param_path = '/action_provider/objects_left/';
    var right_param_path = '/action_provider/objects_right/';

    if (e.target.tagName == 'BUTTON')
    {
        var obj = e.target.firstChild.nodeValue;
        console.log('Pressed '+ e.target.tagName +
                    ' item: ' + obj);

        if (obj == 'error')
        {
          var message = new ROSLIB.Message({
            state: 1,
            isInputOnly: true
          });

          errorPressedL.publish(message);
          errorPressedR.publish(message);
        }
        else if(obj.includes("get_") || obj.includes("c_"))
        {
            var req = new ROSLIB.ServiceRequest();
            var res = new ROSLIB.ServiceResponse();

            req.objects = [];

            // remove prefix, so that we can use the name for other things
            var o = obj.replace(/(get|c)_/g,'');

            var params = new ROSLIB.Param({
                ros: ros,
                name: ""
            });

            // Figure out param name based on the name of buttons
            if(obj.includes("table") || obj.includes("leg")){
                params.name = left_param_path + o;
            }
            else{
                params.name = right_param_path + o;
            }


            req.action = obj.includes("get_")? "get_pass":"cleanup";

            console.log("PARAM: " + params.name);

            // get id associated with each obj and then pick it up
            params.get(function(val){

                req.objects[0] = Number(val);
                console.log("service: " + params.name + " val: " + val);

                console.log('Requested: ', req.action, req.objects);
                // ids < 100 usually are handled by right arm
                if(Number(val) < 100){
                    rightArmService.callService(req,function(res) {
                        console.log('Got Response: ' + res.success);
                    });
                }
                else{
                    leftArmService.callService(req,function(res) {
                        console.log('Got Response: ' + res.success);
                    });
                }
            });

        }
        else if (obj.includes("hold"))
        {
            var req = new ROSLIB.ServiceRequest();
            var res = new ROSLIB.ServiceResponse();

            req.objects = [];

            req.action = obj.includes("top") ? "hold_top": "hold_leg";

            console.log('Requested: ', req.action, req.objects);
            rightArmService.callService(req,function(res) {
                console.log('[right] Got Response: ' + res.success + ' ' + res.response);
            });
        }
        else if (obj == 'home')
        {
          var req = new ROSLIB.ServiceRequest();
          var res = new ROSLIB.ServiceResponse();
          req.action = obj;

          console.log('Requested: ', req.action, req.objects);
          rightArmService.callService(req,function(res)
          {
              console.log('[right] Got Response: ' + res.success + ' ' + res.response);
          });
          leftArmService.callService(req,function(res)
          {
              console.log('[left] Got Response: ' + res.success + ' ' + res.response);
          });
        }

        // Begins rosbagging baxter data
        else if (obj.includes("START"))
        {
            var req = new ROSLIB.ServiceRequest();
            var res = new ROSLIB.ServiceResponse();

            console.log("Requested rosbag recording start!");
            startExperiment.callService(req,function(res){
                console.log("Got response!");
            });
        }

        else if (obj.includes("STOP"))
        {
            var req = new ROSLIB.ServiceRequest();
            var res = new ROSLIB.ServiceResponse();

            console.log("Requested rosbag recording stop!");
            stopExperiment.callService(req,function(res){
                console.log("Got response!");
            });
        }

        else
           {
          if (obj == 'finish') obj = 'stop'; // To comply with the py code

          var message = new ROSLIB.Message({
            data: obj
          });

          elemPressed.publish(message);
        }

        var datenow = Date.now();
        var elapsed = (datenow - beginning)/1000;
        var timestamp = '[' + datenow + '][' + elapsed + ']';
        var logstring = timestamp + ' ' + obj;

        // console.log(logstring);
        var message = new ROSLIB.Message({
          data: logstring
        });
        logTopic.publish(message);
    }

    return;
}

if (document.addEventListener)
    document.addEventListener('click', callback, false);
else
    document.attachEvent('onclick', callback);

