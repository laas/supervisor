rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: false, adreamMocapHuman: false, toasterSimuHuman: true, pr2Robot: true, spencerRobot: false, toasterSimuRobot: false, toasterSimuObject: true}" 

rosservice call /database_manager/execute "{command: 'EMPTY', type: 'ALL', facts: [], agent: '', order: '', areaTopic: false ,agentTopic: false ,move3dTopic: false ,pdgTopic: false}"

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

rosservice call /toaster_simu/add_entity "{id: 'RED_CUBE1', name: 'RED_CUBE1', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'RED_CUBE1', ownerId: '', type: 'object', pose:
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
  {x: 4.5,
  y: 3.7,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'BLUE_CUBE1', name: 'BLUE_CUBE1', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'BLUE_CUBE1', ownerId: '', type: 'object', pose:
{position:
  {x: 4.5,
  y: 3.5,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'BLUE_CUBE2', name: 'BLUE_CUBE2', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'BLUE_CUBE2', ownerId: '', type: 'object', pose:
{position:
  {x: 5.1,
  y: 4.5,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'RED_CUBE2', name: 'RED_CUBE2', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'RED_CUBE2', ownerId: '', type: 'object', pose:
{position:
  {x: 5.2,
  y: 4.6,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'STICK', name: 'STICK', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'STICK', ownerId: '', type: 'object', pose:
{position:
  {x: 5.2,
  y: 4.4,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'PLACEMAT_1', name: 'PLACEMAT_1', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'PLACEMAT_1', ownerId: '', type: 'object', pose:
{position:
  {x: 4.8,
  y: 4.013,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'PLACEMAT_2', name: 'PLACEMAT_2', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'PLACEMAT_2', ownerId: '', type: 'object', pose:
{position:
  {x: 4.8,
  y: 4.187,
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

rosservice call /area_manager/add_area "myArea:
  id: 1
  name: 'stackArea'
  myOwner: 'TABLE_4'
  areaType: ''
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 2.0
  poly:
    points:
    - {x: -0.1, y: -0.2, z: 0}
    - {x: 0.1, y: -0.2, z: 0}
    - {x: 0.1, y: 0.2, z: 0}
    - {x: -0.1, y: 0.2, z: 0}
    - {x: -0.1, y: -0.2, z: 0}
  insideEntities: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 2
  name: 'robotReachable'
  myOwner: 'pr2'
  areaType: ''
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 2.0
  poly:
    points:
    - {x: 0, y: -1, z: 0}
    - {x: 0.75, y: -1, z: 0}
    - {x: 0.75, y: 1, z: 0}
    - {x: 0, y: 1, z: 0}
    - {x: 0, y: -1, z: 0}
  insideEntities: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 3
  name: 'humanReachable'
  myOwner: 'HERAKLES_HUMAN1'
  areaType: ''
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 2.0
  poly:
    points:
    - {x: 0, y: -1, z: 0}
    - {x: 1.25, y: -1, z: 0}
    - {x: 1.25, y: 1, z: 0}
    - {x: 0, y: 1, z: 0}
    - {x: 0, y: -1, z: 0}
  insideEntities: [0]"


rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE1', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE2', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE1', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE2', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'STICK', targetId: 'STICK', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'PLACEMAT_1', targetId: 'AREA', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'PLACEMAT_2', targetId: 'AREA', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'AGENTX', targetId: 'X', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'AGENTX2', targetId: 'X', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'HERAKLES_HUMAN1', targetId: 'AGENT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'PR2_ROBOT', targetId: 'AGENT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'OMNI', targetId: 'OMNI', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'genericName', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE1', targetId: 'RED_CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'genericName', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE2', targetId: 'RED_CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'genericName', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE', targetId: 'GREEN_CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'genericName', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE1', targetId: 'BLUE_CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'genericName', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE2', targetId: 'BLUE_CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'genericName', propertyType: 'state', subProperty: '', subjectId: 'STICK', targetId: 'STICK', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'genericName', propertyType: 'state', subProperty: '', subjectId: 'PLACEMAT_1', targetId: 'PLACEMAT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'genericName', propertyType: 'state', subProperty: '', subjectId: 'PLACEMAT_2', targetId: 'PLACEMAT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'value', propertyType: 'state', subProperty: '', subjectId: 'FTRUE', targetId: 'true', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'value', propertyType: 'state', subProperty: '', subjectId: 'FFALSE', targetId: 'false', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"
