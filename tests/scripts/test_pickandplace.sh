rosservice call /database_manager/execute "{command: 'EMPTY', type: 'ALL', facts: [], agent: '', order: '', areaTopic: false ,agentTopic: false ,move3dTopic: false ,pdgTopic: false}"

rosservice call /pdg/remove_from_hand "objectId: 'LOTR_TAPE'"

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

rosservice call /toaster_simu/add_entity "{id: 'LOTR_TAPE', name: 'LOTR_TAPE', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'LOTR_TAPE', ownerId: '', type: 'object', pose:
{position:
  {x: 4.6,
  y: 3.6,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
      {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'LOTR_TAPE', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
      , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'TABLE_4', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
   ], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

