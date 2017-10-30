var NetworkParser = require('../dist/index.js');
var fs = require('fs');
var assert = require('chai').assert;
var sampleGeoJSON = JSON.parse(fs.readFileSync(__dirname + '/fixtures/sample.geojson', 'utf8')).features;

describe('toPBF()', function() {
	it('should convert network to protobuf', function() {
		var network = new NetworkParser(sampleGeoJSON);
		network.parse({ tolerance: 5 / 111000 });
		var expectedJSON = network.toJSON();
		var pbf = network.toPBF();
		var restoredNetwork = NetworkParser.fromPBF(pbf);
		var restoredJSON = restoredNetwork.toJSON();

		var actualNode = restoredJSON.nodes[5];
		var expectedNode = expectedJSON.nodes[5];
		assert.closeTo(actualNode.coordinates[0], expectedNode.coordinates[0], 1 / 111000);
		assert.deepEqual(actualNode.edgeIds, expectedNode.edgeIds);

		var actualEdge = restoredJSON.edges[5];
		var expectedEdge = expectedJSON.edges[5];
		assert.closeTo(actualEdge.distance, expectedEdge.distance, 0.000001);
		assert.closeTo(actualEdge.cost, expectedEdge.cost, 0.000001);
		assert.equal(actualEdge.nodeIdA, expectedEdge.nodeIdA);
		assert.equal(actualEdge.nodeIdB, expectedEdge.nodeIdB);
		assert.equal(actualEdge.parentFeatureId, expectedEdge.parentFeatureId);
	});
});
