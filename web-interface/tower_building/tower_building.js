var defaultjsonfile = 'towers.json';

loadhtm('');

function loadhtm(file)
{
  if (file == '') { file = defaultjsonfile;}
  else            { defaultjsonfile = file;};

  console.log('Loading file',file);

  var width  = 1200,
      height =  660;

  var i = 0,
      duration = 500,
      rectW = 200,
      rectH = 200;

  var tree = d3.layout.tree()
               .nodeSize([rectW+20, rectH])
               .separation(function separation(a, b) {
                  return (a.parent == b.parent ? 1 : 1.4);
                });

  var diagonal = d3.svg.diagonal()
                   .projection(function(d) { return [d.x+rectW/2, d.y+rectH/2]; });

  var svg = d3.select('svg')
              //responsive SVG needs these 2 attributes and no width and height attr
              .attr('preserveAspectRatio', 'xMinYMin meet')
              .attr('viewBox', '0 0 ' + width + ' ' + height)
              //class to make it responsive
              .classed('svg-content-responsive', true);

  svg.call(d3.behavior.zoom().scaleExtent([0.4, 3]).on('zoom', redraw));

  var vis = svg.append('svg:g');

  var draw = vis.append('svg:g')
                .attr('transform', 'translate(' + 100 + ',' + 100 + ')');

  // load the external data
  d3.json('json/'+file, function(error, json)
  {
    towers = json.towers;

    update(towers);
  });

  function update(source) {

    var blocks = draw.selectAll('g.blocks')
                     .data(source);

    var block = blocks.enter()
                      .append('g')
                      .attr('transform', function(d,i) { return 'translate(' + i*(rectW+100) + ',' + 0 + ')'; });

    var part= block.selectAll('g')
                   .data(function(d,i) {return d.blocks;});

    block.append('rect')
             .attr('width', rectW)
             .attr('height', rectH)
             .attr('class', 'label')
             .style("fill", function(d) { return d.color; });

    block.append('text')
             .attr('x', rectW / 2)
             .attr('y', rectH / 2)
             .attr('text-anchor', 'middle')
             .text(function (d) { return d.blocks; });
  };

  //Redraw for zoom
  function redraw() {
    vis.attr('transform',
             'translate(' + d3.event.translate + ')'
              + ' scale(' + d3.event.scale + ')');
  };


  function click(d) {
    console.log('Pressed item: '+d.name+'\tdepth: '+d.depth+'\tattr: '+d.attributes);
    // console.log(tree.links(d).toString());

    var message = new ROSLIB.Message({
      data: d.name+' '+d.state
    });

    // And finally, publish.
    elemPressed.publish(message);

    if (d.children) {
      d._children = d.children;
      d.children = null;
    } else {
      d.children = d._children;
      d._children = null;
    }
    update(d);
  }

};
