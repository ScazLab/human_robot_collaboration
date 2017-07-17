var width  = 640,
    height = 480;

var obj_dict = [{
    200: "table top",
    150: "leg 1",
    151: "leg 2",
    152: "leg 3",
    153: "leg 4",
    1: "screwdriver",
    2: "screws box",
    3: "brackets box"
}];
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




leftAruco.subscribe(function(msg){

    var left_markers = msg.markers;
    console.log('Received msg on ' + leftAruco.name + ': ' + msg.markers);

    left_svg.selectAll("*").remove();

    var left_objs = left_svg.selectAll("g")
        .data(left_markers).enter().append("g");
    
    var lineFunction = d3.svg.line()
        .x(function(d) {
            return d.x; })
        .y(function(d) { return d.y; })
        .interpolate("linear");

    // UPDATES existing objects-------------------
    left_objs
        .append("text")
        .text(function(d){
            return obj_dict[d.id] == undefined? "Unknown " +d.id : obj_dict[d.id];
        })
        .attr("x", function(d){
            //console.log("X cord of item is :" + d.center.x);
            return d.center.x;})
        .attr("y", function(d){
            //console.log("y cord of item is :" + d.center.y);
            return d.center.y;});


    for(var d in left_markers){
        left_objs.append("path")
            .attr("d", lineFunction(left_markers[d].corners) + " z")
            .attr("stroke", "blue")
            .attr("stroke-width", 2)
            .attr("fill", "none");
    }
});


rightAruco.subscribe(function(msg){

    var right_markers = msg.objects;
    console.log('Received msg on ' + rightAruco.name + ': ' + msg);

    right_svg.selectAll("*").remove();

    var right_objs = right_svg.selectAll("g")
        .data(right_markers).enter().append("g");
    
    var lineFunction = d3.svg.line()
        .x(function(d) {
            return d.x; })
        .y(function(d) { return d.y; })
        .interpolate("linear");

    // UPDATES existing objects-------------------
    right_objs
        .append("text")
        .text(function(d){
            return d.name;
        })
        .attr("x", function(d){
            console.log("X cord of item is :" + d.center.x);
            return d.center.x;})
        .attr("y", function(d){
            console.log("y cord of item is :" + d.center.y);
            return d.center.y;});


    for(var d in right_markers){
        right_objs.append("path")
            .attr("d", lineFunction(right_markers[d].corners) + " z")
            .attr("stroke", "blue")
            .attr("stroke-width", 2)
            .attr("fill", "none");
    }



});



// Draw the Circle
// var circle = left_svg.append("circle")
//                          .attr("cx", 30)
//                          .attr("cy", 30)
//                          .attr("r", 20);

