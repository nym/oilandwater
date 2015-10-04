#
#	test_helper - Set up the test environment
#
#
#
do(Box2D  = require("../lib")) ->

  Object.defineProperties @,

    # Use chai 'should' semantics
    should: value: require('chai').should()

    # The ash framework
    Box2D: value: Box2D

