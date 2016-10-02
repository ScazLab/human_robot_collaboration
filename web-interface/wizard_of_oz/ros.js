// Connecting to ROS
// -----------------
//
var appname = 'woz';
var beginning = Date.now();
var ros = new ROSLIB.Ros();

function resetDate() {
  beginning = Date.now();
  setrobotanswer('');
  setrobotstate('start');
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

  if (msg.data == 'start')
  {
    resetDate();
    setrobotstate('start');
    console.log('Date reset.');
  }
  else
  {
    var data = JSON.parse(msg.data);
    updatetowerscheme(data.towers);
  }
});

var robotStateSub = new ROSLIB.Topic({
  ros : ros,
  name: '/action_provider/state_left',
  messageType: 'baxter_collaboration/ArmState'
})

robotStateSub.subscribe(function(msg) {
  console.log('Received message on ' + robotStateSub.name + ': ' + msg.state);
  setrobotstate(msg.state);
})

function setrobotstate(t) {
  document.getElementById("robotstate").innerHTML = t;
  document.getElementById('robotstate').className = t;
}

var robotAnswerSub = new ROSLIB.Topic({
  ros : ros,
  name: '/web_interface/pub',
  messageType: 'std_msgs/String'
})

robotAnswerSub.subscribe(function(msg) {
  console.log('Received message on ' + robotAnswerSub.name + ': ' + msg.data);
  setrobotanswer(msg.data);
})

var errorStateSub = new ROSLIB.Topic({
  ros : ros,
  name: '/robot/digital_io/left_lower_button/state',
  messageType: 'baxter_core_msgs/DigitalIOState'
})

errorStateSub.subscribe(function(msg) {
  if (msg.state == 1) {
    console.log('Received message on ' + errorStateSub.name + ': error');
    setrobotanswer('error');
  }
})

function setrobotanswer(t) {
  document.getElementById("robotanswer").innerHTML = t;
  document.getElementById('robotanswer').className = t;
}

// Add a callback for any element on the page
function callback(e) {
    var e = window.e || e;

    // console.log(e.target.tagName);
    if (e.target.tagName == 'BUTTON')
    {
        var obj = e.target.firstChild.nodeValue;
        var cls = e.target.className.replace('btn ','').replace(' btn-lg','')
                                    .replace('btn-','').replace('top ','')
                                    .replace('bottom ','').replace('wood ','')
                                    .replace('white ','').replace('blue ','');

        console.log('Pressed '+ e.target.tagName +
                    ' item: ' + obj + ' ' + cls);

        if (cls == 'info')
        {
          setrobotanswer('');
          var req = new ROSLIB.ServiceRequest();
          req.mode = 5;

          var block_type = parseInt(obj)%3==0?'top':'bottom';

          var tower_type = 'wood';
          var tower_num  = Math.floor((parseInt(obj)-1)/3);
          if (tower_num==1) { tower_type = 'white'}
          else if (tower_num==2) { tower_type = 'blue'}

          req.string = 'Do you need the ' + block_type + ' of ' + tower_type + ' tower?';

          console.log('Requested: ', req.string);
          svoxService.callService(req,function(res)
          {
            console.log('Got Response: ' + res.success);
          });
        }
        else if (cls == 'warning')
        {
          setrobotanswer('');
          var req = new ROSLIB.ServiceRequest();
          req.action =    'get_pass';
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
        var logstring = timestamp + '[' + appname + ']' + ' ' + obj;

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

