(define (domain blocksworld)
  (:requirements :strips :equality)
  (:predicates (clear ?x)
               (on-table ?x)
               (arm-empty)
               (holding ?x)
               (on ?x ?y))

  (:action pickup
    :parameters (?ob)
    :precondition (and (clear ?ob) (on-table ?ob) (arm-empty))
    :effect (and (holding ?ob) (not (clear ?ob)) (not (on-table ?ob))
                 (not (arm-empty))))

  (:action putdown
   :parameters  (?ob)
    :precondition (and (holding ?ob))
    :effect (and (clear ?ob) (arm-empty) (on-table ?ob)
                 (not (holding ?ob))))
  ;(:action place
  ;  :parameters (?b ?p ?q)
  ;  :precondition (and (Block ?b) (Kin ?q ?p)
  ;                     (AtConf ?q) (Holding ?b) (not (Unsafe ?p)))
  ;  :effect (and (AtPose ?b ?p) (HandEmpty) (CanMove)
  ;               (not (Holding ?b))
  ;               (increase (total-cost) 1))
  ;)

  (:action stack
    :parameters  (?ob ?underob)
    :precondition (and  (clear ?underob) (holding ?ob))
    :effect (and (arm-empty) (clear ?ob) (on ?ob ?underob)
                 (not (clear ?underob)) (not (holding ?ob))))

  (:action unstack
    :parameters  (?ob ?underob)
    :precondition (and (on ?ob ?underob) (clear ?ob) (arm-empty))
    :effect (and (holding ?ob) (clear ?underob)
                 (not (on ?ob ?underob)) (not (clear ?ob)) (not (arm-empty)))))
  ;(:derived (Unsafe ?p)
  ;  (exists (?b2 ?p2) (and (Pose ?p) (Block ?b2) (Pose ?p2)
  ;                        (not (CFree ?p ?p2))
  ;                        (AtPose ?b2 ?p2)))
  ;)
