describe 'Smoke test: ' , ->

  describe 'verify the api', ->

    it "class module", ->

      Box2D.should.have.property 'Common'
      Box2D.should.have.property 'Collision'
      Box2D.should.have.property 'Dynamics'

  describe 'create objects', ->

    it 'create world', ->

      b2Vec2                = Box2D.Common.Math.b2Vec2
      b2World               = Box2D.Dynamics.b2World

      world = new b2World(new b2Vec2(0 ,0), true)

      (world is null).should.equal(false)

