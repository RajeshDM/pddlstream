(define (problem unity_1)
(:domain unity_1) 
 (:objects
q600
trophy_1
sturdy_box_small
p0
('PickCost',)
1
('PlaceCost',)
treasure_chest_large
p1
bb45d4ea-cf3b-46b5-be23-f4097b8db1c5
p2
brown_box
p3
new_box
p4
p5
trophy
p6

)
(:init

(CanMove)
(BConf q600)
(AtBConf q600)
(Openable trophy_1)
(Openable sturdy_box_small)
(Region p0)
(= ('PickCost',))
(= ('PlaceCost',))
(Pose treasure_chest_large)
(AtPose treasure_chest_large)
(Pickable treasure_chest_large)
(Pose bb45d4ea-cf3b-46b5-be23-f4097b8db1c5)
(AtPose bb45d4ea-cf3b-46b5-be23-f4097b8db1c5)
(Pickable bb45d4ea-cf3b-46b5-be23-f4097b8db1c5)
(Pose brown_box)
(AtPose brown_box)
(Pickable brown_box)
(Pose new_box)
(AtPose new_box)
(Pickable new_box)
(Pose sturdy_box_small)
(AtPose sturdy_box_small)
(Pickable sturdy_box_small)
(Pose trophy)
(AtPose trophy)
(Pickable trophy)
)
(:goal
( and 
(t r)
)
)
)