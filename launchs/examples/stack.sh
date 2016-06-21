rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: false, adreamMocapHuman: false, toasterSimuHuman: true, pr2Robot: true, spencerRobot: false, toasterSimuRobot: false, toasterSimuObject: true}" 

rosservice call /database/execute "{command: 'EMPTY', type: 'ALL', facts: [], agent: '', order: '', areaTopic: false ,agentTopic: false ,move3dTopic: false ,pdgTopic: false}"

rosservice call /toaster_simu/add_entity "{id: 'TABLE_4', name: 'TABLE_4', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'TABLE_4', ownerId: '', type: 'object', pose:
{position:
  {x: 4.8,
  y: 4.1,
  z: -0.1},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'RED_CUBE', name: 'RED_CUBE', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'RED_CUBE', ownerId: '', type: 'object', pose:
{position:
  {x: 4.6,
  y: 3.6,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'GREEN_CUBE', name: 'GREEN_CUBE', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'GREEN_CUBE', ownerId: '', type: 'object', pose:
{position:
  {x: 4.7,
  y: 3.7,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'BLUE_CUBE', name: 'BLUE_CUBE', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'BLUE_CUBE', ownerId: '', type: 'object', pose:
{position:
  {x: 5.1,
  y: 4.5,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'BLACK_CUBE', name: 'BLACK_CUBE', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'BLACK_CUBE', ownerId: '', type: 'object', pose:
{position:
  {x: 5.2,
  y: 4.6,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'PLACEMAT_RED', name: 'PLACEMAT_RED', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'PLACEMAT_RED', ownerId: '', type: 'object', pose:
{position:
  {x: 4.8,
  y: 4.1,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'HERAKLES_HUMAN1', name: 'HERAKLES_HUMAN1', type: 'human', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'HERAKLES_HUMAN1', ownerId: '', type: 'human', pose:
{position:
  {x: 6.0,
  y: 4.1,
  z: 0.0},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 1.0,
  w: 0.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'rightHand', name: 'rightHand', type: 'joint', ownerId: 'HERAKLES_HUMAN1'}"
rosservice call /toaster_simu/set_entity_pose "{id: 'rightHand', ownerId: 'HERAKLES_HUMAN1', type: 'joint', pose:
{position:
  {x: 6.1,
  y: 4.45,
  z: 1.0},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 1.0,
  w: 0.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'base', name: 'base', type: 'joint', ownerId: 'HERAKLES_HUMAN1'}"
rosservice call /toaster_simu/set_entity_pose "{id: 'base', ownerId: 'HERAKLES_HUMAN1', type: 'joint', pose:
{position:
  {x: 6.0,
  y: 4.1,
  z: 0.0},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 1.0,
  w: 0.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'head', name: 'head', type: 'joint', ownerId: 'HERAKLES_HUMAN1'}"
rosservice call /toaster_simu/set_entity_pose "{id: 'head', ownerId: 'HERAKLES_HUMAN1', type: 'joint', pose:
{position:
  {x: 6.0,
  y: 4.1,
  z: 1.5},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 1.0,
  w: 0.0}}}"

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
    - {x: 0, y: -1, z: 0}
    - {x: 2, y: -2, z: 0}
    - {x: 2, y: 2, z: 0}
    - {x: 0, y: 1, z: 0}
    - {x: 0, y: -1, z: 0}
  insideEntities: [0]"

rosservice call /database/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'type', propertyType: 'state', subProperty: '', subjectId: RED_CUBE'', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'BLACK_CUBE', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'TABLE_4', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'PLACEMAT_RED', targetId: 'AREA', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

