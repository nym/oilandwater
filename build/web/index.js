(function init() {
    var kount = 0;
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
	var particleNumber=16;
	var particleDistance=50;

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
	//addBox(320,480,640,20,b2Body.b2_staticBody,false);
	//addBox(320,340,320,20,b2Body.b2_staticBody,false);
	//addBox(170,230,20,200,b2Body.b2_staticBody,false);
	//addBox(470,230,20,200,b2Body.b2_staticBody,false);
	
	// this is the water (is Sensor)
	addBox(300,245,480,470,b2Body.b2_staticBody,true);

	floor();
	sphereVector=[];

	sphereVector.push(sphere(blobX,blobY,15));
	for (var i=0; i<particleNumber; i++) {
		var angle=(2*Math.PI)/particleNumber*i;
		var posX=blobX+particleDistance*Math.cos(angle);
		var posY=blobY+particleDistance*Math.sin(angle);
		sphereVector.push(sphere(posX,posY,2));
		var dJoint=new b2DistanceJointDef();
		dJoint.bodyA=sphereVector[0];
		dJoint.bodyB=sphereVector[sphereVector.length-1];
		dJoint.localAnchorA=new b2Vec2(0,0);
		dJoint.localAnchorB=new b2Vec2(0,0);
		dJoint.length=particleDistance/worldScale;
		dJoint.dampingRatio=0.5;
		dJoint.frequencyHz=5;
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
	/*var waterCanvas=new Sprite();
	addChild(waterCanvas);
	waterCanvas.graphics.beginFill(0x0000ff,0.2);
	waterCanvas.graphics.drawRect(180,160,280,170);
	waterCanvas.graphics.endFill();
*/
	document.addEventListener("keypress", function(e){
		if (e.keyCode == 97) {
			console.log("L");
			console.log(sphereVector[0]);
			if (true) {//if the hero is pressing against the side of a block, applying impulse in the x-dir makes him "stick" to it
				if (sphereVector[0].GetLinearVelocity().x > -10) {//if we haven't reached the max speed in this direction
					sphereVector[0].ApplyImpulse(new b2Vec2( -1, 0), sphereVector[0].GetWorldCenter());
				}
			}
		}
		if (e.keyCode == 100) {
				if (sphereVector[0].GetLinearVelocity().x < 10) {
					sphereVector[0].ApplyImpulse(new b2Vec2( 1, 0), sphereVector[0].GetWorldCenter());
				}
		}
	});
	document.addEventListener("mousedown",function(e){
		//addBox(e.clientX-canvasPosition.x,e.clientY-canvasPosition.y,Math.random()*40,Math.random()*30,b2Body.b2_dynamicBody,false);
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
		fixtureDef.friction=0.5;
		var theSphere=world.CreateBody(bodyDef);
		theSphere.CreateFixture(fixtureDef);
		return theSphere;
	}
	
	function floor() {
		var bodyDef=new b2BodyDef();
		bodyDef.position.Set(320/worldScale,465/worldScale);
		var polygonShape=new b2PolygonShape();
		polygonShape.SetAsBox(320/worldScale,15/worldScale);
		var fixtureDef=new b2FixtureDef();
		fixtureDef.shape=polygonShape;
		fixtureDef.restitution=0.4;
		fixtureDef.friction=0.5;
		var theFloor=world.CreateBody(bodyDef);
		theFloor.CreateFixture(fixtureDef);
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
		fixtureDef.density=1+Math.random();
		fixtureDef.restitution=0.4;
		fixtureDef.friction=0.5;
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

