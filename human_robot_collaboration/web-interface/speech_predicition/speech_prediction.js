var width  = 640,
    height = 480;

var obj_dict = {
    200: "table top",
    150: "leg 1",
    151: "leg 2",
    152: "leg 3",
    153: "leg 4",
    0: "brackets box",
    1: "screwdriver",
    2: "screws box"
};
// Create a left and right SVG, corresponding to the left and right tables
// that baxter picks items up from
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


// Function for drawing polygon from corner coordinates
var lineFunction = d3.svg.line()
    .x(function(d) {
        return d.x; })
    .y(function(d) { return d.y; })
    .interpolate("linear");

// Visualizes aruco data from left arm
leftAruco.subscribe(function(msg){

    var left_markers = msg.markers;
    arucoCallback(left_markers,left_svg,"left");
});


rightAruco.subscribe(function(msg){

    var  right_markers = msg.objects;
    arucoCallback(right_markers,right_svg, "right");
});


leftArmInfo.subscribe(function(msg){
    armInfoCallback(msg, "left");
});

rightArmInfo.subscribe(function(msg){
    armInfoCallback(msg, "right");
});

speech2Text.subscribe(function(msg){
    speech2TextCallback(msg);
});

function armInfoCallback(msg,arm){

    console.log("Receiving arm info!: " + msg.state);
    var s = d3.select("#" + arm + "_baxter_state");
    var a = d3.select("#" + arm + "_baxter_action");
    var o = d3.select("#" + arm + "_baxter_object");

    s.select("text")
        .text("STATE: " + msg.state);

    a.select("text")
        .text("ACTION: " + msg.action);

    o.select("text")
        .text("OBJECT: " + msg.object);
}

function speech2TextCallback(msg){

    var s = d3.select("#speech2Text");

    s.select("text")
        .text("" + msg.transcript);
}

function arucoCallback(markers,s, camera){

    s.selectAll("*").remove();

    var right_objs = s.selectAll("g")
        .data(markers).enter().append("g");

    // UPDATES existing objects-------------------
    right_objs
        .append("text")
        .text(function(d){
            if(camera == "left"){
                return obj_dict[d.id] === undefined? "Unknown " +d.id : obj_dict[d.id];
            }
            else{
                return d.name;
            }
        })
        .attr("x", function(d){
            //console.log("X cord of item is :" + d.center.x);
            return d.center.x;})
        .attr("y", function(d){
            //console.log("y cord of item is :" + d.center.y);
            return d.center.y;})
        .attr("text-anchor", "middle");


    for(var d in markers){
        right_objs.append("path")
            .attr("d", lineFunction( markers[d].corners) + " z")
            .attr("stroke", "blue")
            .attr("stroke-width", 2)
            .attr("fill", "none");
    }
    return;
}
