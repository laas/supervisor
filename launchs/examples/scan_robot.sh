rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: false, adreamMocapHuman: true, toasterSimuHuman: true, pr2Robot: true, spencerRobot: false, toasterSimuRobot: false, toasterSimuObject: true, arObject: true}"

rosservice call /database_manager/execute "{command: 'EMPTY', type: 'ALL', facts: [], agent: '', order: '', areaTopic: false ,agentTopic: false ,move3dTopic: false ,pdgTopic: false}"

rosservice call /agent_monitor/monitor_all_agents "monitorAll: true"

rosservice call /agent_monitor/add_joint_to_agent "{jointName: 'base', agentId: 'HERAKLES_HUMAN1'}"
rosservice call /agent_monitor/add_joint_to_agent "{jointName: 'head', agentId: 'HERAKLES_HUMAN1'}"
rosservice call /agent_monitor/add_joint_to_agent "{jointName: 'rightHand', agentId: 'HERAKLES_HUMAN1'}"

rosservice call /agent_monitor/add_joint_to_agent "{jointName: 'laser_tilt_mount_link', agentId: 'pr2'}"

rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'interaction'
  myOwner: 'pr2'
  areaType: ''
  factType: 'interaction'
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 2.0
  poly:
    points:
    - {x: -2, y: 2, z: 0}
    - {x: -2, y: -2, z: 0}
    - {x: 2, y: -2, z: 0}
    - {x: 2, y: 2, z: 0}
    - {x: -2, y: 2, z: 0}
  insideEntities: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 1
  name: 'scanArea'
  myOwner: 'TABLE_4'
  areaType: ''
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 2.0
  poly:
    points:
    - {x: -0.36, y: -0.7, z: 0}
    - {x: 0.36, y: -0.7, z: 0}
    - {x: 0.36, y: 0.7, z: 0}
    - {x: -0.36, y: 0.7, z: 0}
    - {x: -0.36, y: -0.7, z: 0}
  insideEntities: [0]"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE2', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE3', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE', targetId: 'blue', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE2', targetId: 'blue', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE3', targetId: 'blue', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'PINK_TRASHBIN', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'BLUE_TRASHBIN', targetId: 'blue', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'canScan', propertyType: 'state', subProperty: '', subjectId: 'PR2_ROBOT', targetId: 'true', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"
