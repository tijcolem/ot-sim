module.exports = function(RED) {
  "use strict";
  var zmq = require('zeromq');

  function OTsimIn(config) {
    RED.nodes.createNode(this, config);

    this.tag     = config.tag;
    this.updates = config.updates;

    var node = this;

    var endpoint = 'tcp://localhost:5678';

    if (RED.settings.otsim && RED.settings.otsim.pub) {
      endpoint = RED.settings.otsim.pub;
    }

    var sock = new zmq.Subscriber();

    (async function() {
      await sock.connect(endpoint);
      sock.subscribe('RUNTIME');

      node.status({fill: "green", shape: "ring", text: "subscribing"});

      for await (const [topic, msg] of sock) {
        var parsed = JSON.parse(msg.toString());

        if (parsed.kind === 'Status') {
          for (const m of parsed.contents.measurements) {
            if (m.tag === node.tag) {
              node.send({topic: node.tag, payload: m.value});
            }
          }
        }

        if (node.updates && parsed.kind === 'Update') {
          for (const u of parsed.contents.updates) {
            if (u.tag === node.tag) {
              node.send({payload: u.value});
            }
          }
        }
      }
    })().catch(function(err) {
      if (!sock.closed) {
        node.error(err);
      }
    });

    node.on('close', function(done) {
      sock.close();
      done();
    });
  }

  RED.nodes.registerType("ot-sim in", OTsimIn);

  function OTsimOut(config) {
    RED.nodes.createNode(this, config);

    this.tag = config.tag;
    var node = this;

    var endpoint = 'tcp://localhost:1234';

    if (RED.settings.otsim && RED.settings.otsim.pull) {
      endpoint = RED.settings.otsim.pull;
    }

    var sock = new zmq.Push();
    sock.linger = 0;

    sock.connect(endpoint).then(function() {
      node.status({fill: "yellow", shape: "ring", text: "idle"});
    });

    node.on('input', function(msg) {
      var value = parseFloat(msg.payload);

      if (isNaN(value)) {
        console.log('payload was not a valid floating point number');
        return;
      }

      var update = {
        version: 'v1',
        kind: 'Update',
        metadata: {
          sender: 'Node-RED'
        },
        contents: {
          updates: [
            {
              tag:   node.tag,
              value: value,
              ts:    0.0
            }
          ],
          recipient: '',
          confirm:   ''
        }
      };

      node.status({fill: "green", shape: "ring", text: "updating"});
      sock.send(['RUNTIME', JSON.stringify(update)]).then(function() {
        node.status({fill: "yellow", shape: "ring", text: "idle"});
      }).catch(function(err) {
        node.error(err);
        node.status({fill: "red", shape: "dot", text: "error"});
      });
    });

    node.on('close', function(done) {
      sock.close();
      done();
    });
  }

  RED.nodes.registerType("ot-sim out", OTsimOut);
}
