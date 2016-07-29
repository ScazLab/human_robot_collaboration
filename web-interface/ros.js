// Connecting to ROS
// -----------------
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


// Create a connection to the rosbridge WebSocket server.
ros.connect('ws://localhost:9090');

// Publishing a Topic
// ------------------

// First, we create a Topic object with details of the topic's name and message type.
var elemPressed = new ROSLIB.Topic({
  ros : ros,
  name : '/web_interface',
  messageType : 'std_msgs/String'
});

// // Add a callback for any element on the page
// function callback(e) {
//     var e = window.e || e;

//     // console.log(e.target.tagName);
//     if (e.target.tagName == 'BUTTON')
//     {
//         console.log('Pressed '+ e.target.tagName +
//                     ' item: ' + e.target.firstChild.nodeValue);

//         var message = new ROSLIB.Message({
//           data: e.target.firstChild.nodeValue
//         });

//         elemPressed.publish(message);
//     }

//     return;
// }

// if (document.addEventListener)
//     document.addEventListener('click', callback, false);
// else
//     document.attachEvent('onclick', callback);

