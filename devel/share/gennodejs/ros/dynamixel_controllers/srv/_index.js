
"use strict";

let TorqueEnable = require('./TorqueEnable.js')
let StartController = require('./StartController.js')
let RestartController = require('./RestartController.js')
let StopController = require('./StopController.js')
let SetSpeed = require('./SetSpeed.js')
let SetTorqueLimit = require('./SetTorqueLimit.js')
let SetComplianceSlope = require('./SetComplianceSlope.js')
let SetComplianceMargin = require('./SetComplianceMargin.js')
let SetCompliancePunch = require('./SetCompliancePunch.js')

module.exports = {
  TorqueEnable: TorqueEnable,
  StartController: StartController,
  RestartController: RestartController,
  StopController: StopController,
  SetSpeed: SetSpeed,
  SetTorqueLimit: SetTorqueLimit,
  SetComplianceSlope: SetComplianceSlope,
  SetComplianceMargin: SetComplianceMargin,
  SetCompliancePunch: SetCompliancePunch,
};
