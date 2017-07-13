
var width = screen.width;
var height = screen.height;
var tbl_w = width / 4;
var tbl_left_x = 20.;
var tbl_right_x = 3 * width / 4;

var svg = d3.select("svg");

var data = [{label: "Click me 1", x: width / 4, y: height / 4 },
            {label: "Click me 2", x: width / 2, y: height / 2 },
            {label: "Click me 3", x: width / 4, y: height / 2 },
            {label: "Click me 4", x: width / 2, y: height / 4 }];

var tbl_data = [{label: "Left Table", x: tbl_left_x , y: 20, w: width / 4, h: height},
                {label: "Right Table",x: tbl_right_x , y: 20, w: width / 4, h: height}];


var tbls = svg.selectAll(".table").data(tbl_data)
    .enter()
    .append("g");

tbls.append("rect")
    .attr("width", function(d){return d.w;})
    .attr("height",  function(d){return d.h;})
    .attr("x",function(d){return d.x;})
    .attr("y",function(d){return d.y;})
    .style("fill", " #2a1614");


var title = tbls.append("text")
    .text( function (d) { return d.label; })
    .attr("x",function (d) { return d.x + d.w / 3; })
    .attr("y",function (d) { return d.y + 30; })
    .attr("fill", "red")
    .attr("font-family", "sans-serif")
    .attr("font-size", "10");


var numButtonCols = data.length / 2;

// Add buttons


var sel = svg.selectAll(".button")
    .data(data)
    .enter().append("g");

var textLabels = sel.append("text")
    .text( function (d) { return d.label; })
    .attr("font-family", "sans-serif")
    .attr("font-size", "10");

var bbox = textLabels.node().getBBox();

textLabels
    .attr("x", function(d, i){return (tbl_w + 150 + ( 50 + bbox.width) *(i % 2));})
    .attr("y", function(d, i){return d.y;});

// collect the bounding box of each button in array
var bboxes  = [];
textLabels.each(function(){

    bboxes.push(this.getBBox());
});

var offset = 5;

var  ClickEvent = function(){
    console.log("YOU CLICKED");
};

var MouseOver = function(d){
    d3.select(this).style("fill", "#fa5111");
};

var MouseOut = function(d){
    d3.select(this).style("fill", "#fa1f11");
};

sel.append("rect")
    .attr("class","button")
    .attr("id", function(d,i){return "button"+i;})
    .attr("width", bbox.width  + offset)
    .attr("height", bbox.height + offset)
    .attr("x", function(d,i){return bboxes[i].x - offset / 2;})
    .attr("y", function(d,i){return bboxes[i].y - offset / 2;})
    .style("fill", "#fa1f11")
    .style("fill-opacity", ".3")
    .on({"click": ClickEvent,
         "mouseover": MouseOver,
         "mouseout": MouseOut,
         "mousedown": function(){
             d3.select(this).attr("border", 1)
                 .style("stroke", "black");
         },
         "mouseup": function(){
             d3.select(this).attr("border", 1)
                 .style("stroke", "none");
         }
        });
