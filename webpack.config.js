const path = require('path');
const nodeExternals = require('webpack-node-externals');

module.exports = {
	target: 'node',
	entry: './index.js',
	output: {
		path: path.resolve(__dirname, 'dist'),
		filename: 'index.js',
		libraryTarget: 'commonjs-module',
	},
	externals: [nodeExternals({
		whitelist: ['node-dijkstra']
	})],
	module: {
		rules: [
			{
				test: /\.js$/,
				use: {
					loader: 'babel-loader',
					options: {
						presets: ['es2015']
					}
				}
			}
		]
	}
};
