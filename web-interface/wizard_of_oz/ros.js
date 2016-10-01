// Connecting to ROS
// -----------------
//
var beginning = Date.now();
var ros = new ROSLIB.Ros();

function resetDate() {
  beginning = Date.now();
}

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
var webInterfacePub = new ROSLIB.Topic({
  ros : ros,
  name : '/wizard_of_oz/pub',
  messageType : 'std_msgs/String'
});

// Service Client to interface with the left arm
var leftArmService  = new ROSLIB.Service({
  ros : ros,
  name: '/action_provider/service_left',
  messageType : 'baxter_collaboration/DoAction'
});

// Service Client to interface with the right arm
var svoxService = new ROSLIB.Service({
  ros : ros,
  name: '/svox_tts/speech',
  messageType : 'svox_tts/Speech'
});

var webInterfaceSub = new ROSLIB.Topic({
  ros : ros,
  name: '/wizard_of_oz/sub',
  messageType: 'std_msgs/String'
});

webInterfaceSub.subscribe(function(msg) {
  console.log('Received message on ' + webInterfaceSub.name);
  var data = JSON.parse(msg.data);
  updatetowerscheme(data.towers);
});

var robotStateSub = new ROSLIB.Topic({
  ros : ros,
  name: '/action_provider/state_left',
  messageType: 'baxter_collaboration/ArmState'
})

robotStateSub.subscribe(function(msg) {
  console.log('Received message on ' + robotStateSub.name + ': ' + msg.state);
  var data
  document.getElementById("robotstate").innerHTML = msg.state;
})

// Add a callback for any element on the page
function callback(e) {
    var e = window.e || e;

    // console.log(e.target.tagName);
    if (e.target.tagName == 'BUTTON')
    {
        var obj = e.target.firstChild.nodeValue;
        var cls = e.target.className.replace('btn ','').replace(' btn-lg','').replace('btn-','');

        console.log('Pressed '+ e.target.tagName +
                    ' item: ' + obj + ' ' + cls);

        if (cls == 'info')
        {
          var req = new ROSLIB.ServiceRequest();
          req.mode = 5;

          var block_type = parseInt(obj)%2==1?'bottom':'top';

          var tower_type = 'blue';
          var tower_num  = Math.floor((parseInt(obj)-1)/2);
          if (tower_num==1) { tower_type = 'wood'}
          else if (tower_num==2) { tower_type = 'white'}

          req.string = 'Do you need the ' + block_type + ' of ' + tower_type + ' tower?';

          console.log('Requested: ', req.string);
          svoxService.callService(req,function(res)
          {
            console.log('Got Response: ' + res.success);
          });
        }
        else if (cls == 'warning')
        {
          var req = new ROSLIB.ServiceRequest();
          req.action =    'pass_get';
          req.object = parseInt(obj);

          var res = new ROSLIB.ServiceResponse();

          console.log('Requested: ', req.action, req.object);
          leftArmService.callService(req,function(res)
          {
            console.log('Got Response: ' + res.success);
          });
        }
        else
        {
          if (obj == 'finish') obj = 'stop'; // To comply with the py code

          var message = new ROSLIB.Message({
            data: obj
          });

          webInterfacePub.publish(message);
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

