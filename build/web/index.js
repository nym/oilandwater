(function init() {
	var b2Vec2 = Box2D.Common.Math.b2Vec2;
	var b2AABB = Box2D.Collision.b2AABB;
	var b2BodyDef = Box2D.Dynamics.b2BodyDef;
	var b2Body = Box2D.Dynamics.b2Body;
	var b2FixtureDef = Box2D.Dynamics.b2FixtureDef;
	var b2Fixture = Box2D.Dynamics.b2Fixture;
	var b2World = Box2D.Dynamics.b2World;
	var b2CircleShape = Box2D.Collision.Shapes.b2CircleShape;
	var b2DistanceJointDef = Box2D.Dynamics.Joints.b2DistanceJointDef;
	var b2DistanceJoint = Box2D.Dynamics.Joints.b2DistanceJoint;

	var b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape;
	var b2DebugDraw = Box2D.Dynamics.b2DebugDraw;
    var b2BuoyancyController = Box2D.Dynamics.Controllers.b2BuoyancyController;
	var sphereVector;
	var blobX=320;
	var blobY=240;
	var particleNumber=9;
	var particleDistance=60;

    var canvas = document.createElement(navigator.isCocoonJS ? 'screencanvas' : 'canvas');
    canvas.width  = window.innerWidth*window.devicePixelRatio;
    canvas.height = window.innerHeight*window.devicePixelRatio;
    canvas.style.width = '100%';
    canvas.style.height = '100%';
    document.body.appendChild(canvas);

    var ctx = canvas.getContext("2d");
  
	var worldScale = 20;
	
	var world = new b2World(new b2Vec2(0, 10),true);
	var buoyancyController = new b2BuoyancyController();
	var canvasPosition = {x:0, y:0};  
	
	debugDraw();             
	window.setInterval(update,1000/30);
	
	buoyancyController.normal.Set(0,-1);
	buoyancyController.offset=-180/worldScale;
	buoyancyController.useDensity=true;
	buoyancyController.density=2.0;
	buoyancyController.linearDrag=5;
	buoyancyController.angularDrag=2;
	world.AddController(buoyancyController);
	
	// this is the water (is Sensor)
	addBox(300,300,5080,5070,b2Body.b2_staticBody,true);

	floor();
	ceiling();
	left();
	right();
	sphereVector=[];

	sphereVector.push(sphere(blobX,blobY,15));
	for (var i=0; i<particleNumber; i++) {
		var angle=(2*Math.PI)/particleNumber*i;
		var posX=blobX+particleDistance*Math.cos(angle);
		var posY=blobY+particleDistance*Math.sin(angle);
		sphereVector.push(sphere(posX,posY,3));
		var dJoint=new b2DistanceJointDef();
		dJoint.bodyA=sphereVector[0];
		dJoint.bodyB=sphereVector[sphereVector.length-1];
		dJoint.localAnchorA=new b2Vec2(0,0);
		dJoint.localAnchorB=new b2Vec2(0,0);
		dJoint.length=particleDistance/worldScale;
		dJoint.dampingRatio=0.6;
		dJoint.frequencyHz=4;
		var distanceJoint;
		distanceJoint=world.CreateJoint(dJoint);
		if (i>0) {
			var distanceX=posX/worldScale-sphereVector[sphereVector.length-2].GetPosition().x;
			var distanceY=posY/worldScale-sphereVector[sphereVector.length-2].GetPosition().y;
			var distance=Math.sqrt(distanceX*distanceX+distanceY*distanceY);
			dJoint.bodyA=sphereVector[sphereVector.length-2];
			dJoint.bodyB=sphereVector[sphereVector.length-1];
			dJoint.localAnchorA=new b2Vec2(0,0);
			dJoint.localAnchorB=new b2Vec2(0,0);
			dJoint.length=distance;
			distanceJoint=world.CreateJoint(dJoint);
		}
		if (i==particleNumber-1) {
			distanceX=posX/worldScale-sphereVector[1].GetPosition().x;
			distanceY=posY/worldScale-sphereVector[1].GetPosition().y;
			distance=Math.sqrt(distanceX*distanceX+distanceY*distanceY);
			dJoint.bodyA=sphereVector[1];
			dJoint.bodyB=sphereVector[sphereVector.length-1];
			dJoint.localAnchorA=new b2Vec2(0,0);
			dJoint.localAnchorB=new b2Vec2(0,0);
			dJoint.length=distance;
			distanceJoint=world.CreateJoint(dJoint);
		}
	}

	document.addEventListener("keypress", function(e){
		if (e.keyCode == 97) {
			if (true) {//if the hero is pressing against the side of a block, applying impulse in the x-dir makes him "stick" to it
				if (sphereVector[0].GetLinearVelocity().x > -40) {//if we haven't reached the max speed in this direction
					sphereVector[0].ApplyImpulse(new b2Vec2( -4, 0), sphereVector[0].GetWorldCenter());
					var s = bubble(sphereVector[0].GetPosition().x+4, sphereVector[0].GetPosition().y, 10);
					s.ApplyImpulse(new b2Vec2( 5, Math.random()-0.5), s.GetWorldCenter());
				}
			}
		}
		if (e.keyCode == 100) {
				if (sphereVector[0].GetLinearVelocity().x < 40) {
					sphereVector[0].ApplyImpulse(new b2Vec2( 4, 0), sphereVector[0].GetWorldCenter());
					var s = bubble(sphereVector[0].GetPosition().x-4, sphereVector[0].GetPosition().y, 10);
					s.ApplyImpulse(new b2Vec2( -5, Math.random()-0.5), s.GetWorldCenter());

				}
		}
		d = sphereVector[0].m_fixtureList.m_density;
		if (e.keyCode == 115 ) {
			if (sphereVector[0].GetLinearVelocity().y > -20) {

				sphereVector[0].ApplyImpulse(new b2Vec2( 0, 2), sphereVector[0].GetWorldCenter());
				sphereVector[0].m_fixtureList.SetDensity(d+0.05);
				sphereVector[0].ResetMassData();
				var s = bubble(sphereVector[0].GetPosition().x, sphereVector[0].GetPosition().y-4, 10);
				s.ApplyImpulse(new b2Vec2( Math.random()-0.5, -5), s.GetWorldCenter());

			}
		}
		if (e.keyCode == 119 ) {
			if (sphereVector[0].GetLinearVelocity().y < 20) {

				sphereVector[0].ApplyImpulse(new b2Vec2( 0, -4), sphereVector[0].GetWorldCenter());
				sphereVector[0].m_fixtureList.SetDensity(d-0.05);
				sphereVector[0].ResetMassData();
				var s = bubble(sphereVector[0].GetPosition().x, sphereVector[0].GetPosition().y+4, 10);
				s.ApplyImpulse(new b2Vec2( Math.random()-0.5, 5), s.GetWorldCenter());

			}
		}

	});

	
	function sphere(pX,pY,r) {
		var bodyDef=new b2BodyDef();
		bodyDef.position.Set(pX/worldScale,pY/worldScale);
		bodyDef.type=b2Body.b2_dynamicBody;
		var circleShape;
		circleShape=new b2CircleShape(r/worldScale);
		var fixtureDef=new b2FixtureDef();
		fixtureDef.shape=circleShape;
		fixtureDef.density=1;
		fixtureDef.restitution=0.4;
		fixtureDef.friction=0.4;
		var theSphere=world.CreateBody(bodyDef);
		theSphere.CreateFixture(fixtureDef);
		return theSphere;
	}
	
	function bubble(pX,pY) {
		var bodyDef=new b2BodyDef();
		bodyDef.position.Set(pX,pY);
		bodyDef.type=b2Body.b2_dynamicBody;
		var circleShape;
		circleShape=new b2CircleShape(10*Math.random()/worldScale);
		var fixtureDef=new b2FixtureDef();
		fixtureDef.shape=circleShape;
		fixtureDef.isSensor=true;
		fixtureDef.density=0.9;
		fixtureDef.restitution=0.4;
		fixtureDef.friction=0 ;
		var theSphere=world.CreateBody(bodyDef); 
		theSphere.CreateFixture(fixtureDef);
		setTimeout(function(){world.DestroyBody(theSphere)}, 1500);
		return theSphere;
	}

	document.addEventListener("mousedown",function(e){
		fluff(e.clientX * 1.3,e.clientY*1.3,Math.random()*10,Math.random()*10,b2Body.b2_dynamicBody,false);
	});
	
	function floor() {
		var bodyDef=new b2BodyDef();

		bodyDef.position.Set(899/worldScale,899/worldScale);
		var polygonShape=new b2PolygonShape();
		polygonShape.SetAsBox(5000/worldScale,15/worldScale);
		var fixtureDef=new b2FixtureDef();
		fixtureDef.shape=polygonShape;
		fixtureDef.restitution=0.4;
		fixtureDef.friction=0.5;
		var theFloor=world.CreateBody(bodyDef);
		theFloor.CreateFixture(fixtureDef);
	}

	function left() {
		var bodyDef=new b2BodyDef();

		bodyDef.position.Set(10/worldScale,10/worldScale);
		var polygonShape=new b2PolygonShape();
		polygonShape.SetAsBox(20/worldScale,1000/worldScale);
		var fixtureDef=new b2FixtureDef();
		fixtureDef.shape=polygonShape;
		fixtureDef.restitution=0.4;
		fixtureDef.friction=0.5;
		var theFloor=world.CreateBody(bodyDef);
		theFloor.CreateFixture(fixtureDef);


	}
	function right() {
		var bodyDef=new b2BodyDef();

		bodyDef.position.Set(1699/worldScale,100/worldScale);
		var polygonShape=new b2PolygonShape();
		polygonShape.SetAsBox(10/worldScale,1100/worldScale);
		var fixtureDef=new b2FixtureDef();
		fixtureDef.shape=polygonShape;
		fixtureDef.restitution=0.4;
		fixtureDef.friction=0.5;
		var theFloor=world.CreateBody(bodyDef);
		theFloor.CreateFixture(fixtureDef);
	}

	function ceiling() {
		var bodyDef=new b2BodyDef();
		bodyDef.position.Set(1/worldScale,10/worldScale);
		var polygonShape=new b2PolygonShape();
		polygonShape.SetAsBox(5000/worldScale,15/worldScale);
		var fixtureDef=new b2FixtureDef();
		fixtureDef.shape=polygonShape;
		fixtureDef.restitution=0.4;
		fixtureDef.friction=0.5;
		var theFloor=world.CreateBody(bodyDef);
		theFloor.CreateFixture(fixtureDef);
	}

	function fluff(pX,pY,w,h,bodyType,isSensor) {
		var bodyDef=new b2BodyDef();
		bodyDef.position.Set(pX/worldScale,pY/worldScale);
		bodyDef.type=b2Body.b2_dynamicBody;
		var circleShape;
		circleShape=new b2CircleShape(w/worldScale);
		var fixtureDef=new b2FixtureDef();
		fixtureDef.shape=circleShape;
		fixtureDef.isSensor=true;
		fixtureDef.density=0.5+Math.random();
		fixtureDef.restitution=0.4;
		fixtureDef.friction=0.10;
		var theSphere=world.CreateBody(bodyDef);
		theSphere.CreateFixture(fixtureDef);
		setTimeout(function(){world.DestroyBody(theSphere)}, 10000);
		return theSphere;
	}

	function addBox(pX,pY,w,h,bodyType,isSensor){
		var bodyDef=new b2BodyDef();
		bodyDef.position.Set(pX/worldScale,pY/worldScale);
		bodyDef.type=bodyType;
		var polygonShape=new b2PolygonShape();
		polygonShape.SetAsBox(w/2/worldScale,h/2/worldScale);
		var fixtureDef=new b2FixtureDef();
		fixtureDef.isSensor=isSensor;
		fixtureDef.shape=polygonShape;
		fixtureDef.density=0.5+Math.random();
		fixtureDef.restitution=0.4;
		fixtureDef.friction=0.1;
		var body=world.CreateBody(bodyDef);
		body.CreateFixture(fixtureDef);
	}
	
	function debugDraw(){
		var debugDraw = new b2DebugDraw();
		debugDraw.SetSprite(ctx);
		debugDraw.SetDrawScale(30.0);
		debugDraw.SetFillAlpha(0.5);
		debugDraw.SetLineThickness(1.0);
		debugDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit);
		world.SetDebugDraw(debugDraw);
	}
	
	function update() {
        world.Step(1/30,10,10);
        fluff(Math.random() * 10000,Math.random() * 10000,Math.random()*10,Math.random()*10,b2Body.b2_dynamicBody,false);

		for (var currentBody=world.GetBodyList(); currentBody; currentBody=currentBody.GetNext()) {
			if (currentBody.GetType()==b2Body.b2_dynamicBody) {
				var currentBodyControllers=currentBody.GetControllerList();
				if (currentBodyControllers!=null) {
					buoyancyController.RemoveBody(currentBody);
				}
				for (var c=currentBody.GetContactList(); c; c=c.next) {
					var contact=c.contact;
					var fixtureA=contact.GetFixtureA();
					var fixtureB=contact.GetFixtureB();
					if (fixtureA.IsSensor()) {
						var bodyB=fixtureB.GetBody();
						var bodyBControllers=bodyB.GetControllerList();
						if (bodyBControllers==null) {
							buoyancyController.AddBody(bodyB);
						}
					}
					if (fixtureB.IsSensor()) {
						var bodyA=fixtureA.GetBody();
						var bodyAControllers=bodyA.GetControllerList();
						if (bodyAControllers==null) {
							buoyancyController.AddBody(bodyA);
						}
					}
				}
			}
		}
		world.ClearForces();
		world.DrawDebugData();
	}
	
})();
