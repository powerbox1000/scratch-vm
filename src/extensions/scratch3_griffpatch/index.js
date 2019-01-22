// https://cdn.jsdelivr.net/gh/griffpatch/griffpatch.github.io/testExtension.js

const ArgumentType = require('../../extension-support/argument-type');
const BlockType = require('../../extension-support/block-type');
// const MathUtil = require('../../util/math-util');
// const Clone = require('../../util/clone');
const Cast = require('../../util/cast');
const Runtime = require('../../engine/runtime')
const formatMessage = require('format-message');
// const MathUtil = require('../../util/math-util');
// const Timer = require('../../util/timer');
// const Matter = require('matterJs/matter');
// const Matter = require('matter-js');

// const Box2D = require('./Box2d.min').box2d;
const Box2D = require('./box2d_es6');

// window.decomp = require('poly-decomp');

const b2Vec2 = Box2D.Common.Math.b2Vec2;
const b2AABB = Box2D.Collision.b2AABB;
const b2BodyDef = Box2D.Dynamics.b2BodyDef;
const b2Body = Box2D.Dynamics.b2Body;
const b2FixtureDef = Box2D.Dynamics.b2FixtureDef;
const b2Fixture = Box2D.Dynamics.b2Fixture;
const b2World = Box2D.Dynamics.b2World;
const b2MassData = Box2D.Collision.Shapes.b2MassData;
const b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape;
const b2CircleShape = Box2D.Collision.Shapes.b2CircleShape;
const b2DebugDraw = Box2D.Dynamics.b2DebugDraw;
const b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef;
const b2Math = Box2D.Common.Math.b2Math;

let world; let zoom;

const fixDef = new b2FixtureDef();
const bodyDef = new b2BodyDef();

const uid_seq = 0;
let ujidSeq = 0;

const prevPos = {};
const bodies = {};
const joints = {};

const categorySeq = 1;
const categories = {default: 1};

const bodyCategoryBits = 1;
const bodyMaskBits = 1;
const noCollideSeq = 0;

const toRad = Math.PI / 180;

const _definePolyFromHull = function (hullPoints) {
    fixDef.shape = new b2PolygonShape();

    const vertices = [];

    let prev = null;
    for (let i = hullPoints.length - 1; i >= 0; i--) {
        const b2Vec = new b2Vec2(hullPoints[i].x / zoom, hullPoints[i].y / zoom);
        if (prev !== null && b2Math.SubtractVV(b2Vec, prev).LengthSquared() > Number.MIN_VALUE) {
            vertices.push(b2Vec);
        }
        prev = b2Vec;
    }

    fixDef.shape.SetAsArray(vertices);
};

const _placeBody = function (id, x, y, dir) {
    if (bodies[id]) {
        world.DestroyBody(bodies[id]);
    }

    fixDef.filter.categoryBits = bodyCategoryBits;
    fixDef.filter.maskBits = bodyMaskBits;

    bodyDef.position.x = x / zoom;
    bodyDef.position.y = y / zoom;
    bodyDef.angle = (90 - dir) * toRad;

    const body = world.CreateBody(bodyDef);
    body.uid = id;
    body.CreateFixture(fixDef);
    bodies[id] = body;
};

const _applyForce = function (id, ftype, x, y, dir, pow) {
    const body = bodies[id];
    if (!body) {
        return;
    }

    dir = (90 - dir) * toRad;

    if (ftype === 'Impulse') {

        const center = body.GetLocalCenter(); // get the mass data from you body

        body.ApplyImpulse({x: pow * Math.cos(dir), y: pow * Math.sin(dir)},
            body.GetWorldPoint({x: (x / zoom) + center.x, y: (y / zoom) + center.y}));
    } else if (ftype === 'World Impulse') {
        body.ApplyForce({x: pow * Math.cos(dir), y: pow * Math.sin(dir)},
            {x: x / zoom, y: y / zoom});
    }
};

// ['', 'Define Spring Length: %n Damping: %n  Freq: %n', '_defineSpring', 100, 0.5, 8],
const defSpring = {len: 100, damp: 0.7, freq: 5};
const _defineSpring = function (len, damp, freq) {
    defSpring.len = len < 0.1 ? 0.1 : len / zoom;
    defSpring.damp = damp < 0 ? 0.7 : damp;
    defSpring.freq = freq > 0 ? freq : 5;
};

const _createJointOfType = function (jName, typ, bodyID, x, y, bodyID2, x2, y2) {

    // if (jName.length > 0) ext.destroyJoint(jName);

    if (!bodyID) bodyID = null;
    if (!bodyID2) bodyID2 = null;
    if (!bodyID && !bodyID2) {
        return null;
    }

    const body = bodyID ? bodies[bodyID] : world.GetGroundBody();
    const body2 = bodyID2 ? bodies[bodyID2] : world.GetGroundBody();

    if (!body || !body2) return null;

    let md;
    switch (typ) {
    case 'Spring':
        md = new Box2D.Dynamics.Joints.b2DistanceJointDef();
        md.length = defSpring.len;
        md.dampingRatio = defSpring.damp;
        md.frequencyHz = defSpring.freq;
        md.bodyA = body;
        md.bodyB = body2;
        md.localAnchorA = {x: x / zoom, y: y / zoom};
        md.localAnchorB = {x: x2 / zoom, y: y2 / zoom};
        break;

    case 'Rotating':
        md = new Box2D.Dynamics.Joints.b2RevoluteJointDef();
        md.bodyA = body;
        md.bodyB = body2;
        md.localAnchorA = {x: x / zoom, y: y / zoom};
        if (x2 === null) {
            md.localAnchorB = body.GetWorldPoint({x: (x / zoom), y: (y / zoom)});
        } else {
            md.localAnchorB = {x: x2 / zoom, y: y2 / zoom};
        }
        break;

    case 'Mouse':
        md = new b2MouseJointDef();
        if (bodyID) {
            md.bodyB = body;
            md.target.Set(x / zoom, y / zoom);
        } else {
            md.bodyB = body2;
            md.target.Set(x2 / zoom, y2 / zoom);
        }
        md.bodyA = world.GetGroundBody();
        md.collideConnected = true;
        md.maxForce = 300.0 * body.GetMass();
        break;
    }

    // md.collideConnected = true;
    // md.maxForce = 300.0 * body.GetMass();
    const joint = world.CreateJoint(md);
    if (bodyID) {
        body.SetAwake(true);
    }
    if (bodyID2) {
        body2.SetAwake(true);
    }

    if (!jName) jName = '_' + (++ujidSeq);
    joints[jName] = joint;
};

/**
 * Class for the music-related blocks in Scratch 3.0
 * @param {Runtime} runtime - the runtime instantiating this block package.
 * @constructor
 */
class Scratch3Griffpatch {

    constructor (runtime) {
        /**
         * The runtime instantiating this block package.
         * @type {Runtime}
         */
        this.runtime = runtime;

        // Clear target motion state values when the project starts.
        this.runtime.on(Runtime.PROJECT_START, this.reset.bind(this));

        world = new b2World(
            new b2Vec2(0, -10), // gravity (10)
            true // allow sleep
        );

        zoom = 50; // scale;

        this.map = {};

        fixDef.density = 1.0; // 1.0
        fixDef.friction = 0.5; // 0.5
        fixDef.restitution = 0.2; // 0.2

        bodyDef.type = b2Body.b2_staticBody;
        fixDef.shape = new b2PolygonShape();
        fixDef.shape.SetAsBox(250 / zoom, 10 / zoom);
        bodyDef.position.Set(0, -190 / zoom);
        world.CreateBody(bodyDef).CreateFixture(fixDef);
        bodyDef.position.Set(0, 1000 / zoom);
        world.CreateBody(bodyDef).CreateFixture(fixDef);
        fixDef.shape.SetAsBox(10 / zoom, 800 / zoom);
        bodyDef.position.Set(-250 / zoom, 540 / zoom);
        world.CreateBody(bodyDef).CreateFixture(fixDef);
        bodyDef.position.Set(250 / zoom, 540 / zoom);
        world.CreateBody(bodyDef).CreateFixture(fixDef);

        bodyDef.type = b2Body.b2_dynamicBody;
    }

    reset () {
        for (const body in bodies) {
            world.DestroyBody(bodies[body]);
            delete bodies[body];
            delete prevPos[body];
        }

        // todo: delete joins?
    }

    /**
     * The key to load & store a target's music-related state.
     * @type {string}
     */
    static get STATE_KEY () {
        return 'Scratch.Griffpatch';
    }

    /**
     * @returns {object} metadata for this extension and its blocks.
     */
    getInfo () {
        return {
            id: 'griffpatch',
            name: formatMessage({
                id: 'griffpatch.categoryName',
                default: 'Griffpatch',
                description: 'Label for the Griffpatch extension category'
            }),
            // menuIconURI: menuIconURI,
            // blockIconURI: blockIconURI,
            blocks: [
                {
                    opcode: 'setPhysicsAll',
                    blockType: BlockType.COMMAND,
                    text: formatMessage({
                        id: 'griffpatch.setPhysicsAll',
                        default: 'Enable Physics for All Sprites',
                        description: 'Enable Physics For All Sprites'
                    })
                },
                {
                    opcode: 'setPhysics',
                    blockType: BlockType.COMMAND,
                    text: formatMessage({
                        id: 'griffpatch.setPhysics',
                        default: 'Enable Physics for Sprite',
                        description: 'Enable Physics for this Sprite'
                    })
                },
                {
                    opcode: 'setDensity',
                    blockType: BlockType.COMMAND,
                    text: formatMessage({
                        id: 'griffpatch.setDensity',
                        default: 'Set Density [density]',
                        description: 'Set the density of the object'
                    }),
                    arguments: {
                        density: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 1
                        }
                    }
                },

                // applyForce (target, ftype, x, y, dir, pow) {
                // applyAngForce (target, pow) {

                {
                    opcode: 'applyForce',
                    blockType: BlockType.COMMAND,
                    text: formatMessage({
                        id: 'griffpatch.applyForce',
                        default: 'Push with force [force] in direction [dir]',
                        description: 'Push this object in a given direction'
                    }),
                    arguments: {
                        force: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 25
                        },
                        dir: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 0
                        }
                    }
                },
                {
                    opcode: 'applyAngForce',
                    blockType: BlockType.COMMAND,
                    text: formatMessage({
                        id: 'griffpatch.applyAngForce',
                        default: 'Spin with force [force]',
                        description: 'Push this object in a given direction'
                    }),
                    arguments: {
                        force: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 500
                        }
                    }
                },
                {
                    opcode: 'pinSprite',
                    blockType: BlockType.COMMAND,
                    text: formatMessage({
                        id: 'griffpatch.pinSprite',
                        default: 'Pin Sprite at x: [x], y: [y]',
                        description: 'Pin the sprite'
                    }),
                    arguments: {
                        x: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 0
                        },
                        y: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 0
                        }
                    }
                },
                {
                    opcode: 'doTick',
                    blockType: BlockType.COMMAND,
                    text: formatMessage({
                        id: 'griffpatch.doTick',
                        default: 'Step Physics Simulation',
                        description: 'Run a single tick of the physics simulation'
                    })
                },

                {
                    opcode: 'setStatic',
                    blockType: BlockType.COMMAND,
                    text: formatMessage({
                        id: 'griffpatch.setStatic',
                        default: 'Set Static [static]',
                        description: 'Sets whether this block is static or dynamic'
                    }),
                    arguments: {
                        static: {
                            type: ArgumentType.BOOLEAN,
                            defaultValue: true
                        }
                    }
                },
                {
                    opcode: 'getStatic',
                    text: formatMessage({
                        id: 'griffpatch.getStatic',
                        default: 'Static?',
                        description: 'get whether this sprite is static'
                    }),
                    blockType: BlockType.BOOLEAN
                }
            ]
        };
    }

    /**
     * Play a drum sound for some number of beats.
     * @param {object} args - the block arguments.
     * @param {object} util - utility object provided by the runtime.
     * @property {number} x - x offset.
     * @property {number} y - y offset.
     */
    doTick (args, util) {
        // this._playDrumForBeats(args.DRUM, args.BEATS, util);
        // if (util.runtime.audioEngine === null) return;
        // if (util.target.sprite.soundBank === null) return;

        // const dx = Cast.toNumber(args.x);
        // const dy = Cast.toNumber(args.y);

        // const allTargets = this.runtime.targets;
        // if (allTargets === null) return;
        // for (let i = 0; i < allTargets.length; i++) {
        //     const target = allTargets[i];
        //     if (!target.isStage) {
        //         target.setXY(target.x + dx, target.y + dy);
        //     }
        // }

        // util.target.setXY(util.target.x + dx, util.target.y + dy);

        // Matter.Engine.update(this.engine, 1000 / 30);

        for (const targetID in bodies) {
            const body = bodies[targetID];
            const target = this.runtime.getTargetById(targetID);
            if (!target) {
                // Drop target from simulation
                world.DestroyBody(body);
                delete bodies[targetID];
                delete prevPos[targetID];
                continue;
            }

            const prev = prevPos[targetID];
            if (prev && (prev.x !== target.x || prev.y !== target.y || prev.dir !== target.direction)) {
                const pos = new b2Vec2(target.x / zoom, target.y / zoom);
                body.SetPositionAndAngle(pos, (90 - target.direction) * toRad);
                body.SetAwake(true);
            }
        }

        world.Step(1 / 30, 10, 10);
        world.ClearForces();

        for (const targetID in bodies) {
            const body = bodies[targetID];
            const target = this.runtime.getTargetById(targetID);
            if (!target) {
                // Drop target from simulation
                world.DestroyBody(body);
                delete bodies[targetID];
                delete prevPos[targetID];
                continue;
            }

            const position = body.GetPosition();

            target.setXY(position.x * zoom, position.y * zoom);
            target.setDirection(90 - (body.GetAngle() / toRad));

            prevPos[targetID] = {x: target.x, y: target.y, dir: target.direction};
        }
    }

    /**
     * Play a drum sound for some number of beats.
     * @param {object} args - the block arguments.
     * @param {object} util - utility object provided by the runtime.
     * @property {number} x - x offset.
     * @property {number} y - y offset.
     */
    setPhysicsAll (args, util) {

        const allTargets = this.runtime.targets;
        if (allTargets === null) return;
        for (let i = 0; i < allTargets.length; i++) {
            const target = allTargets[i];
            if (!target.isStage) {
                this.setPhysicsFor(target);
            }
        }

    }

    /**
     * Play a drum sound for some number of beats.
     * @param {object} args - the block arguments.
     * @param {object} util - utility object provided by the runtime.
     * @property {number} x - x offset.
     * @property {number} y - y offset.
     */
    setPhysics (args, util) {
        // this._playDrumForBeats(args.DRUM, args.BEATS, util);
        // if (util.runtime.audioEngine === null) return;
        // if (util.target.sprite.soundBank === null) return;

        // const dx = Cast.toNumber(args.x);
        // const dy = Cast.toNumber(args.y);

        const target = util.target;
        this.setPhysicsFor(target);

        // debugger;
    }

    setPhysicsFor (target) {

        const r = this.runtime.renderer;
        const drawable = r._allDrawables[target.drawableID];

        // const svg = drawable._skin._svgRenderer._svgTag;

        // Tell the Drawable about its updated convex hullPoints, if necessary.
        if (drawable.needsConvexHullPoints()) {
            const points = r._getConvexHullPointsForDrawable(target.drawableID);
            drawable.setConvexHullPoints(points);
        }

        // if (drawable._transformDirty) {
        //     drawable._calculateTransform();
        // }
        // const points = drawable._getTransformedHullPoints();
        //
        // const hullPoints = [];
        // for (const i in points) {
        //     hullPoints.push({x: points[i][0] - target.x, y: points[i][1] - target.y});
        // }

        const points = drawable._convexHullPoints;
        const scaleX = drawable.scale[0] / 100;
        const scaleY = drawable.scale[1] / -100; // Flip Y for hulls
        const offset = drawable.skin.rotationCenter;

        const hullPoints = [];
        for (const i in points) {
            hullPoints.push({x: (points[i][0] - offset[0]) * scaleX, y: (points[i][1] - offset[1]) * scaleY});
        }

        _definePolyFromHull(hullPoints);
        _placeBody(target.id, target.x, target.y, target.direction);
    }

    applyForce (args, util) {
        _applyForce(util.target.id, 'Impulse', 0, 0,
            Cast.toNumber(args.dir), Cast.toNumber(args.force));
    }

    applyAngForce (args, util) {
        const body = bodies[util.target.id];
        if (!body) {
            return;
        }

        body.ApplyTorque(-Cast.toNumber(args.force));
    }

    setDensity (args, util) {
        const body = bodies[util.target.id];
        if (!body) {
            return;
        }

        body.GetFixtureList().SetDensity(Cast.toNumber(args.density));
        body.ResetMassData();
    }

    pinSprite (args, util) {
        const body = bodies[util.target.id];
        if (!body) {
            return;
        }

        const x = Cast.toNumber(args.x);
        const y = Cast.toNumber(args.y);

        _createJointOfType(null, 'Rotating', util.target.id, x, y, null, null, null);
    }

    /**
     * Get the current tempo.
     * @return {boolean} - the current tempo, in beats per minute.
     */
    getStatic (args, util) {
        const body = bodies[util.target.id];
        if (!body) {
            return false;
        }
        const type = body.GetType();
        return type === b2Body.b2_staticBody;
    }

    /**
     * Sets the static property
     */
    setStatic (args, util) {
        const target = util.target;
        this.setPhysicsFor(target);
        const body = bodies[util.target.id];
        if (!body) {
            return;
        }
        body.SetType(args.static ? b2Body.b2_staticBody : b2Body.b2_dynamicBody);

        const pos = new b2Vec2(target.x / zoom, target.y / zoom);
        body.SetPositionAndAngle(pos, (90 - target.direction) * toRad);
    }
}

module.exports = Scratch3Griffpatch;
