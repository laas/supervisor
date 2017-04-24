rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: false, adreamMocapHuman: false, toasterSimuHuman: true, pr2Robot: true, spencerRobot: false, toasterSimuRobot: false, toasterSimuObject: true}"

rosservice call /database_manager/execute "{command: 'EMPTY', type: 'ALL', facts: [], agent: '', order: '', areaTopic: false ,agentTopic: false ,move3dTopic: false ,pdgTopic: false}"

rosparam set use_sim_time true

rosservice call /toaster_simu/add_entity "{id: 'TABLE_4', name: 'TABLE_4', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'TABLE_4', ownerId: '', type: 'object', pose:
{position:
  {x: 4.8,
  y: 6.1,
  z: -0.1},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'RED_CUBE1', name: 'RED_CUBE1', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'RED_CUBE1', ownerId: '', type: 'object', pose:
{position:
  {x: 9.0,
  y: 11.0,
  z: 0.82},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'RED_CUBE2', name: 'RED_CUBE2', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'RED_CUBE2', ownerId: '', type: 'object', pose:
{position:
  {x: 4.6,
  y: 5.6,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'RED_CUBE3', name: 'RED_CUBE3', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'RED_CUBE3', ownerId: '', type: 'object', pose:
{position:
  {x: 9.0,
  y: 11.0,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'GREEN_CUBE1', name: 'GREEN_CUBE1', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'GREEN_CUBE1', ownerId: '', type: 'object', pose:
{position:
  {x: 9.0,
  y: 11.0,
  z: 0.94},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'GREEN_CUBE2', name: 'GREEN_CUBE2', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'GREEN_CUBE2', ownerId: '', type: 'object', pose:
{position:
  {x: 4.6,
  y: 5.6,
  z: 0.82},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'GREEN_CUBE3', name: 'GREEN_CUBE3', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'GREEN_CUBE3', ownerId: '', type: 'object', pose:
{position:
  {x: 9.0,
  y: 11.0,
  z: 0.76},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'BLUE_CUBE1', name: 'BLUE_CUBE1', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'BLUE_CUBE1', ownerId: '', type: 'object', pose:
{position:
  {x: 4.6,
  y: 5.6,
  z: 0.88},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'BLUE_CUBE2', name: 'BLUE_CUBE2', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'BLUE_CUBE2', ownerId: '', type: 'object', pose:
{position:
  {x: 9.0,
  y: 11.0,
  z: 0.88},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'BLUE_CUBE3', name: 'BLUE_CUBE3', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'BLUE_CUBE3', ownerId: '', type: 'object', pose:
{position:
  {x: 4.6,
  y: 5.6,
  z: 0.76},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'RED_TAPE1', name: 'RED_TAPE1', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'RED_TAPE1', ownerId: '', type: 'object', pose:
{position:
  {x: 4.8,
  y: 5.4,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"
rosservice call /toaster_simu/add_entity "{id: 'RED_TAPE2', name: 'RED_TAPE2', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'RED_TAPE2', ownerId: '', type: 'object', pose:
{position:
  {x: 6.0,
  y: 11.5,
  z: 0.8},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'SCAN_AREA1', name: 'SCAN_AREA1', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'SCAN_AREA1', ownerId: '', type: 'object', pose:
{position:
  {x: 4.8,
  y: 5.9,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'SCAN_AREA2', name: 'SCAN_AREA2', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'SCAN_AREA2', ownerId: '', type: 'object', pose:
{position:
  {x: 4.8,
  y: 6.3,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"


rosservice call /toaster_simu/add_entity "{id: 'BLUE_BOX', name: 'BLUE_BOX', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'BLUE_BOX', ownerId: '', type: 'object', pose:
{position:
  {x: 4.4,
  y: 7.0,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'GREEN_BOX', name: 'GREEN_BOX', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'GREEN_BOX', ownerId: '', type: 'object', pose:
{position:
  {x: 9.0,
  y: 12.0,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"


rosservice call /toaster_simu/add_entity "{id: 'RED_BOX1', name: 'RED_BOX1', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'RED_BOX1', ownerId: '', type: 'object', pose:
{position:
  {x: 4.0,
  y: 7.0,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'RED_BOX2', name: 'RED_BOX2', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'RED_BOX2', ownerId: '', type: 'object', pose:
{position:
  {x: 9.0,
  y: 11.5,
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
  y: 6.1,
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
  y: 6.45,
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
  y: 6.1,
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
  y: 6.1,
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
    - {x: -1, y: -1.5, z: 0}
    - {x: 1, y: -1.5, z: 0}
    - {x: 1, y: 1.5, z: 0}
    - {x: -1, y: 1.5, z: 0}
    - {x: -1, y: -1.5, z: 0}
  insideEntities: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 3
  name: 'humanReachable1'
  myOwner: 'TABLE_4'
  areaType: ''
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 2.0
  poly:
    points:
    - {x: -0.2, y: -0.3, z: 0}
    - {x: 0.2, y: -0.3, z: 0}
    - {x: 0.2, y: 0.3, z: 0}
    - {x: -0.2, y: 0.3, z: 0}
    - {x: -0.2, y: -0.3, z: 0}
  insideEntities: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 4
  name: 'humanReachable2'
  myOwner: 'RED_BOX2'
  areaType: ''
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 2.0
  poly:
    points:
    - {x: -0.5, y: -1.5, z: 0}
    - {x: 0.5, y: -1.5, z: 0}
    - {x: 0.5, y: 1.5, z: 0}
    - {x: -0.5, y: 1.5, z: 0}
    - {x: -0.5, y: -1.5, z: 0}
  insideEntities: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 5
  name: 'humanReachable3'
  myOwner: 'RED_TAPE2'
  areaType: ''
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 2.0
  poly:
    points:
    - {x: -0.1, y: -0.1, z: 0}
    - {x: 0.1, y: -0.1, z: 0}
    - {x: 0.1, y: 0.1, z: 0}
    - {x: -0.1, y: 0.1, z: 0}
    - {x: -0.1, y: -0.1, z: 0}
  insideEntities: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 6
  name: 'areaVisible'
  myOwner: 'TABLE_4'
  areaType: ''
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 2.0
  poly:
    points:
    - {x: -1, y: -2, z: 0}
    - {x: 5, y: -2, z: 0}
    - {x: 5, y: 2, z: 0}
    - {x: -1, y: 2, z: 0}
    - {x: -1, y: -2, z: 0}
  insideEntities: [0]"


rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE1', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE2', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE3', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE1', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE2', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE3', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE1', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE2', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE3', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'RED_TAPE1', targetId: 'tape', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'RED_TAPE2', targetId: 'tape', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'SCAN_AREA1', targetId: 'area', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'SCAN_AREA2', targetId: 'area', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'AGENTX', targetId: 'X', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'AGENTX2', targetId: 'X', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'HERAKLES_HUMAN1', targetId: 'human', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'PR2_ROBOT', targetId: 'robot', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE1', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE2', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE3', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE1', targetId: 'green', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE2', targetId: 'green', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE3', targetId: 'green', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE1', targetId: 'blue', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE2', targetId: 'blue', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE3', targetId: 'blue', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_TAPE1', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_TAPE2', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_BOX1', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_BOX2', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'BLUE_BOX', targetId: 'blue', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'GREEN_BOX', targetId: 'green', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'HERAKLES_HUMAN1', facts: [
    {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE1', targetId: 'GREEN_CUBE2', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE2', targetId: 'BLUE_CUBE3', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE3', targetId: 'RED_CUBE2', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE1', targetId: 'BLUE_CUBE2', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE2', targetId: 'RED_CUBE1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE1', targetId: 'GREEN_CUBE3', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE3', targetId: 'RED_CUBE3', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE1', targetId: 'GREEN_CUBE2', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE2', targetId: 'BLUE_CUBE3', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE3', targetId: 'RED_CUBE2', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE1', targetId: 'BLUE_CUBE2', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE2', targetId: 'RED_CUBE1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE1', targetId: 'GREEN_CUBE3', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE3', targetId: 'RED_CUBE3', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_BOX2', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_TAPE2', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'value', propertyType: 'state', subProperty: '', subjectId: 'FTRUE', targetId: 'true', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'value', propertyType: 'state', subProperty: '', subjectId: 'FFALSE', targetId: 'false', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'HERAKLES_HUMAN1', facts: [
    {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE1', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE2', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE3', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE2', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'BLUE_BOX', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_BOX1', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_TAPE1', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'SCAN_AREA1', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'SCAN_AREA2', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE1', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE2', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE1', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE3', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE3', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'GREEN_BOX', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_BOX2', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_TAPE2', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'SCAN_AREA1', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'SCAN_AREA2', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: { frame_id: "/map" }, pose: { pose: { position: { x: 4.2, y: 6.2 }, orientation: { x: 0, y: 0, z: 0.0, w: 1.0 } }, covariance: [ 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ] } }'
