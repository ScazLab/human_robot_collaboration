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
    return 'localhost';
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

// Service Client to interface with the left arm
var leftArmService  = new ROSLIB.Service({
  ros : ros,
  name: '/action_provider/service_left',
  messageType : 'baxter_collaboration_msgs/DoAction'
});

// Service Client to interface with the right arm
var rightArmService = new ROSLIB.Service({
  ros : ros,
  name: '/action_provider/service_right',
  messageType : 'baxter_collaboration_msgs/DoAction'
});

// Add a callback for any element on the page
function callback(e) {
    var e = window.e || e;

    // console.log(e.target.tagName);
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
        else if (obj == 'hold' || obj == 'release')
        {
          var req = new ROSLIB.ServiceRequest();

          if      (obj == 'hold')    { req.action = 'start_hold'; }
          else if (obj == 'release') { req.action =   'end_hold'; }

          var res = new ROSLIB.ServiceResponse();

          console.log('Requested: ', req.action, req.objects);
          rightArmService.callService(req,function(res)
          {
              console.log('Got Response: ' + res.success + ' ' + res.response);
          });
        }
        else if (obj == 'get CF' || obj == 'get LL'  ||
                 obj == 'get RL' || obj == 'get TOP' ||
                 obj == 'pass' )
        {
          var req = new ROSLIB.ServiceRequest();
          req.objects = new Array();

          if (obj == 'pass') { req.action = 'pass'; }
          else               { req.action = 'get';  }

          if      (obj.replace('get ','') == 'CF')  { req.objects[0] = 24; }
          else if (obj.replace('get ','') == 'LL')  { req.objects[0] = 17; }
          else if (obj.replace('get ','') == 'RL')  { req.objects[0] = 26; }
          else if (obj.replace('get ','') == 'TOP') { req.objects[0] = 21; }
          else { console.error('Requested an object that was not allowed!'); };

          var res = new ROSLIB.ServiceResponse();

          console.log('Requested: ', req.action, req.objects);
          leftArmService.callService(req,function(res)
          {
            console.log('Got Response: ' + res.success + ' ' + res.response);
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

