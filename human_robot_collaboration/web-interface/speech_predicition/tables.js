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


function left_update(left_markers){

};


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
            console.log("X cord of item is :" + d.center.x);
            return d.center.x;})
        .attr("y", function(d){
            console.log("y cord of item is :" + d.center.y);
            return d.center.y;});


    for(var d in left_markers){
        left_objs.append("path")
            .attr("d", lineFunction(left_markers[d].corners) + " z")
            .attr("stroke", "blue")
            .attr("stroke-width", 2)
            .attr("fill", "none");
    }



    // ADDS newly seen objects-----------
    // var left_g = left_objs
    //     .enter()
    //     .append("g");


    // left_g.append("text")
    //     .text (function(d){
    //         return obj_dict[d.id] == undefined? "Unknown " +d.id : obj_dict[d.id];
    //     })
    //     .attr("x", function(d){
    //         console.log("X cord of item is :" + d.center.x);
    //         return d.center.x;})
    //     .attr("y", function(d){
    //         console.log("y cord of item is :" + d.center.y);
    //         return d.center.y;});


    // left_g .append ("rect")
    //     .attr("width", this.select('text').getBBox().width)
    //     .attr("height", this.select('text').getBBox().height)
    //     .attr("x", function(d,i){return d.center.x;})
    //     .attr("y", function(d,i){return d.center.y;})
    //     .style("fill", "black")
    //     .style("fill-opacity", ".1");
    // REMOVES objects that are no longer seen-----
    //left_objs.exit().remove();

});





// Draw the Circle
// var circle = left_svg.append("circle")
//                          .attr("cx", 30)
//                          .attr("cy", 30)
//                          .attr("r", 20);

