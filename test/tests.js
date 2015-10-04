
describe('Box2DWeb Tests', function() {
    var b2World = Box2D.Dynamics.b2World;
    var b2Body = Box2D.Dynamics.b2Body;
    var b2BodyDef = Box2D.Dynamics.b2BodyDef;
    var b2CircleShape = Box2D.Collision.Shapes.b2CircleShape;
    var b2FixtureDef = Box2D.Dynamics.b2FixtureDef;
    var b2Vec2 = Box2D.Common.Math.b2Vec2;

    it('Box2D API should exist', function(){
        expect(Box2D).to.not.equal(null);
    });

    it('Create 10000 bullets', function(done) {

       var world = new b2World(new b2Vec2(0, 0), true);
       var bullets = [];
       var bodyDef;
       var fixDef;
       var body;

       for (var i=0; i<10000; i++) {
           bodyDef = new b2BodyDef();
           bodyDef.type = b2Body.b2_dynamicBody;
           bodyDef.fixedRotation = true;
           bodyDef.position.x = 100;
           bodyDef.position.y = 100;
           bodyDef.linearVelocity.Set(.01, .01);
           bodyDef.angularVelocity = 0;

           fixDef = new b2FixtureDef();
           fixDef.density = 1.0;
           fixDef.friction = 0.0;
           fixDef.restitution = 0.2;
           fixDef.shape = new b2CircleShape(0);

           body = world.CreateBody(bodyDef);
           body.CreateFixture(fixDef);
           bullets.push(body);
       }
       expect(bullets.length).to.equal(10000);
       this.timeout(10000);
       done();
   });

});

