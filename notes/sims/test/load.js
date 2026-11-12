// Node loader for simulation model tests.
// Usage: const RASim = require('./load.js')('two-link-arm'); const m = RASim.sims['two-link-arm'].model;
//        require('./load.js')('my-sim', '/path/to/my-sim.js')  -- load a sim file from another path
//        require('./load.js')()                                -- core only
const path = require('path');

module.exports = function load(id, file) {
  if (!globalThis.RASim) globalThis.RASim = require('../core.js');
  const RASim = globalThis.RASim;
  if (id && !RASim.sims[id]) {
    const f = file ? path.resolve(file) : path.join(__dirname, '..', id + '.js');
    require(f);
    if (!RASim.sims[id]) throw new Error('load: ' + f + ' did not register a simulation with id "' + id + '"');
  }
  return RASim;
};
