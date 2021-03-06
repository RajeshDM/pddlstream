(define (domain pr2-tamp)
  (:requirements :strips :equality)
  (:predicates
    (Arm ?a)
    (Stackable ?o ?r)
    (Sink ?r)
    (Stove ?r)

    (Grasp ?o ?g)
    (Kin ?a ?o ?p ?g ?q ?t)
    (BaseMotion ?q1 ?t ?q2)
    (ArmMotion ?a ?q1 ?t ?q2)
    (Supported ?o ?p ?r)
    (Supports ?o2 ?p2 ?o1 ?p1)

    (TrajPoseCollision ?t ?o ?p)
    (TrajArmCollision ?t ?a ?q)
    (TrajGraspCollision ?t ?a ?o ?g)

    (Pose ?o ?p)
    ;(Object ?o)
    (AtPose ?o ?p)
    (AtGrasp ?a ?o ?g)
    (HandEmpty ?a)
    (AtBConf ?q)
    (AtAConf ?a ?q)
    (CanMove)
    (Cleaned ?o)
    (Cooked ?o)
    (Open ?o)
    (Openable ?o)
    (OpenableConf ?o ?q)
    (PickedUp ?o)
    (Pickable ?o)
    (PickableConf ?o ?q ?p)
    (DroppableConf ?o ?q ?p)
    (Occupied ?p)
    (UnOccupied ?p)
    (MovableConf ?o ?q)
    (CFree ?p1 ?p2)
    (CFreeObjectPose ?o ?p ?o2 ?p2)
    (CFreeTrajPose ?t ?o ?p)
    (ObjInRegion ?r ?o2 )
    (ObjPoseInRegion ?r ?o2 ?p2)
    ;(ObjInRegion ?o1 ?p1 ?o2 )
    ;(ObjPoseInRegion ?o1 ?p1 ?o2 ?p2)
    ;(CFreeTrajPose ?q1 ?q2)
    ;(NoAgentObjectCollision ?o ?p ?q)

    (On ?o ?r)
    (Holding ?a ?o)
    (IsHolding)
    (UnsafeBTraj ?t)
    (UnsafePose ?o ?p)
    (Unsafe ?p)
    (UnsafePose2 ?o ?p)
    (UnsafeBMove ?t)
  )
  (:functions
    (MoveCost ?t)
    (PickCost)
    (PlaceCost)
  )

  (:action move_base
    :parameters (?q1 ?q2 ?t)
    :precondition (and (BaseMotion ?q1 ?t ?q2)
                       ;(AtBConf ?q1) (CanMove) (not (UnsafeBTraj ?t)))
                       (AtBConf ?q1) (CanMove)
                       ;;(not (UnsafeBMove ?t))
                       )
                       ;(AtBConf ?q1) (CanMove) (not (UnsafeMove ?q1 ?q2)))

    :effect (and (AtBConf ?q2)
                 (not (AtBConf ?q1))
                 (increase (total-cost) (MoveCost ?t)))
  )
  ; (:action move_base)
  ;  :parameters (?q1 ?q2)
  ;  :precondition (and () (AtBConf ?q1) (CanMove))
  ;  :effect (and (AtBconf ?q2) (not (AtBConf ?q1)) )

   (:action open_object
    :parameters (?o ?q)
    :precondition (and
                    (OpenableConf ?o ?q)
                    ;(AtPose ?o ?p)
                    (Openable ?o)
                    (not (Open ?o))
                    (AtBConf ?q)
                  )
    :effect (Open ?o)
    )
   (:action pickup_object
    :parameters (?o ?q ?p)
    :precondition (and
                    (PickableConf ?o ?q ?p)
                    (AtPose ?o ?p)
                    (Pickable ?o)
                    (AtBConf ?q)
                    (not (IsHolding))
                  )
    :effect (and (PickedUp ?o) (IsHolding) (CanMove) (not (AtPose ?o ?p)) )
 )
   (:action drop_object
    :parameters (?o ?q ?p)
    :precondition (and
                    (DroppableConf ?o ?q ?p)
                    (AtBConf ?q)
                    (IsHolding)
                    ;(not (UnsafePose ?o ?p))
                    ;(not (UnsafePose2 ?o ?p))
                    ;(not (Unsafe ?p))
                    (PickedUp ?o)
                  )
    :effect (and (AtPose ?o ?p) (not (IsHolding))  (CanMove) (not (PickedUp ?o)))
 )
   ;(:action move_object
   ; :parameters (?o ?p1 ?p2 ?q)
   ; :precondition (and
   ;                 (MovableConf ?o ?q)
   ;                 (AtPose ?o p1)
   ;                 (UnOccupied ?p2)
   ;                 (AtBConf ?q)
   ;                 ;(NoAgentObjectCollision ?o ?p2 ?q)
   ;               )
   ; :effect (and (AtPose ?o ?p2)
   ;              (not (AtPose ?o ?p1))
   ;              (UnOccupied ?p1)
   ;              (Occupied ?p2)
   ;              )
  ;)
  ;(:action move_arm
  ;  :parameters (?q1 ?q2 ?t)
  ;  :precondition (and (ArmMotion ?a ?q1 ?t ?q2)
  ;                     (AtAConf ?a ?q1))
  ;  :effect (and (AtAConf ?a ?q2)
  ;               (not (AtAConf ?a ?q1)))
  ;)
  (:action pick
    :parameters (?a ?o ?p ?g ?q ?t)
    :precondition (and (Kin ?a ?o ?p ?g ?q ?t)
                       (AtPose ?o ?p) (HandEmpty ?a) (AtBConf ?q))
    :effect (and (AtGrasp ?a ?o ?g) (CanMove)
                 ;(forall (?r) (when (Supported ?o ?p ?r) (not (On ?o ?r))))
                 (not (AtPose ?o ?p)) (not (HandEmpty ?a))
                 (increase (total-cost) (PickCost)))
  )
  ;(:action place
  ;  :parameters (?a ?o ?p ?g ?q ?t)
  ;  :precondition (and (Kin ?a ?o ?p ?g ?q ?t)
  ;                     (AtGrasp ?a ?o ?g) (AtBConf ?q))
  ;  :effect (and (AtPose ?o ?p) (HandEmpty ?a) (CanMove)
  ;               (not (AtGrasp ?a ?o ?g))
  ;               ;(forall (?r) (when (Supported ?o ?p ?r) (On ?o ?r)))
  ;               (increase (total-cost) (PlaceCost)))
  ;)

  ;(:derived (UnsafeBTraj ?t) (or
  ;  (exists (?o2 ?p2) (and (TrajPoseCollision ?t ?o2 ?p2)
  ;                         (AtPose ?o2 ?p2)))
  ;  (exists (?a ?q) (and (TrajArmCollision ?t ?a ?q)
  ;                       (AtAConf ?a ?q)))
  ;  (exists (?a ?o ?g) (and (TrajGraspCollision ?t ?a ?o ?g)
  ;                       (AtGrasp ?a ?o ?g)))
  ;))

  (:derived (UnsafeBMove ?t)
    (exists (?o2 ?p2) (and (not (CFreeTrajPose ?t ?o2 ?p2))
                           ;(not (CFreeTrajPose ?t ?o2 ?p2))
                           (AtPose ?o2 ?p2) (Pickable ?o2)))
  )

   (:derived (Unsafe ?p)
   ;(exists (?o2 ?p2) (and (Pose ?p) (Object ?o2) (Pose o2? ?p2)
   (exists (?o2 ?p2) (and (Object ?o2) (Pose ?o2 ?p2)
                          (not (CFree ?p ?p2))
                          (AtPose ?o2 ?p2)))
   )
    (:derived (UnsafePose ?o ?p)
    (exists (?o2 ?p2) (and (Pose ?o ?p) (Pose ?o2 ?p2) (not (= ?o ?o2))
                           (not (CFreeObjectPose ?o ?p ?o2 ?p2))
                           (AtPose ?o2 ?p2)))
  )
    (:derived (UnsafePose2 ?o ?p)
    (exists (?o2) (and (Pose ?o ?p) (Pose ?o2 ?p) (not (= ?o ?o2))
                           ;(not (CFreeObjectPose ?o ?p ?o2 ?p2))
                           (AtPose ?o2 ?p)
                           ))
  )
    (:derived (ObjInRegion ?r ?o2)
    (exists (?p2) (and (ObjPoseInRegion ?r ?o2 ?p2)
                           (AtPose ?o2 ?p2)
                           ))
    ;(:derived (ObjInRegion ?o1 ?p1 ?o2)
    ;(exists (?p2) (and (ObjPoseInRegion ?o1 ?p1 ?o2 ?p2)
    ;                       (AtPose ?o2 ?p2)
    ;                       ))
    ;(exists (?p2) (and (Pose ?o1 ?p1) (Pose ?o2 ?p2)
    ;                       (ObjPoseInRegion ?o1 ?p1 ?o2 ?p2)
    ;                       ;(Supports ?o2 ?p2 ?o1 ?p1)
    ;                       (AtPose ?o2 ?p2)
    ;                       ))
  )
)