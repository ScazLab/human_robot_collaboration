var defaultjsonfile = 'towers.json';

loadtowerscheme('');

var vis;
var draw;

var i = 0,
    rectW = 140,
    rectH = 140;

function loadtowerscheme(file)
{
  if (file == '') { file = defaultjsonfile;}
  else            { defaultjsonfile = file;};

  console.log('Loading file',file);

  var width  =  900,
      height =  560;

  var svg = d3.select('svg')
              //responsive SVG needs these 2 attributes and no width and height attr
              .attr('preserveAspectRatio', 'xMinYMin meet')
              .attr('viewBox', '0 0 ' + width + ' ' + height)
              //class to make it responsive
              .classed('svg-content-responsive', true);

  svg.call(d3.behavior.zoom().scaleExtent([0.4, 3]).on('zoom', redraw));

  vis  = svg.append('svg:g');
  draw = vis.append('svg:g')
            .attr('transform', 'translate(' + 140 + ',' + 60 + ')');

  // load the external data
  d3.json('json/'+file, function(error, json)
  {
    if (error) {throw error;}
    updatetowerscheme(json.towers);
  });
};

function updatetowerscheme(source) {
  draw.selectAll('*').remove();
  console.log('Updating towers..',source);

  var towers = draw.selectAll('g')
                   .data(source)
                   .enter()
                   .append('g')
                   .attr('class', 'tower')
                   .attr('transform', function(d,i) { return 'translate(' + d.x0*(rectW+100) + ',' + 0 + ')'; });

  var max_length = 0;
  towers.each(function(d,i) {
    if (d.blocks.length>max_length) { max_length=d.blocks.length}
  });

  towers.each(function(d,i) {
    for (var j = 0; j < max_length; j++)
    {
      if (j<d.blocks.length) {
      var block = d3.select(this)
                    .append('g')
                    .attr('transform', 'translate(' + 0 + ',' + ((max_length-1-j)*rectH*(1+0.04)) + ')');

      block.append('rect')
           .attr('width', rectW)
           .attr('height', rectH)
           .attr('class', 'label')
           .style('fill', d.blocks[j].color);

      block.append('text')
           .attr('x', rectW / 2)
           .attr('y', rectH / 2)
           .attr('text-anchor', 'middle')
           .text(d.blocks[j].label);
      }
    }
  });
};

//Redraw for zoom
function redraw() {
  vis.attr('transform',
           'translate(' + d3.event.translate + ')'
            + ' scale(' + d3.event.scale + ')');
};
