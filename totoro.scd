(Material pinkmat
    (color .9 .5 .5)
    (ka .1) (kd .7) (ks .5) (ksp 20) (ksm 0) (kt 0) (ktn 1)
)

(Material bodymat
    (color .39 .59 .99)
    (ka .1) (kd .8) (ks .1) (ksp 4) (ksm 0) (kt 0) (ktn 1)
)

(Material whitemat
    (color 1 1 1)
    (ka .1) (kd .8) (ks .1) (ksp 4) (ksm 0) (kt 0) (ktn 1)
)

(Material graymat
    (color .55 .55 .55)
    (ka .1) (kd .8) (ks .1) (ksp 4) (ksm 0) (kt 0) (ktn 1)
)

(Material blackmat
    (color .2 .2 .2)
    (ka .1) (kd .1) (ks .8) (ksp 150) (ksm 0) (kt 0) (ktn 1)
)

(Material brownmat
    (color .4 .1 .1)
    (ka .1) (kd .1) (ks .8) (ksp 150) (ksm 0) (kt 0) (ktn 1)
)

(Material voidmat
    (color 0 0 0)
    (ka 0) (kd 0) (ks 0) (ksp 1) (ksm 1) (kt 0) (ktn 1)
)

(Material groundmat
    (color .4 .7 .2)
    (ka .1) (kd .6) (ks .3) (ksp 1) (ksm 1) (kt 0) (ktn 1)
)

(Material skymat
    (color 1 1 1)
    (ka .1) (kd .8) (ks .1) (ksp .5) (ksm 1) (kt 0) (ktn 1)
)

(Material sunmat
    (color 1 1 0)
    (ka .1) (kd .8) (ks .1) (ksp .5) (ksm 1) (kt 0) (ktn 1)
)

(Sphere sph1 (radius 1.0) (material bodymat))
(Sphere sph2 (radius 1.0) (material whitemat))
(Sphere sph3 (radius 1.0) (material graymat))
(Sphere sph4 (radius 1.0) (material whitemat))
(Sphere sph5 (radius 1.0) (material blackmat))
(Sphere sph6 (radius 1.0) (material brownmat))

# sky and sun
(Sphere ssph (radius 1.0) (material skymat))
(Sphere ysph (radius 1.0) (material sunmat))

(Light amb
    (type 3) (color .6 .6 .2)
)

(Light dir
    (type 0) (color .9 .9 .9)
)

(Light pt
    (type 1) (color 1.39 1.2 1.0) (falloff 2) (deaddistance .01)
)

(Light sunlt
    (type 1) (color 1.39 1.2 1.0) (falloff 0) (deaddistance .01)
)

(Camera cam
    (perspective 1)
    (l -.334) (r .334) (b -.334) (t .334) (n 1) # no far plane 
)

(G stomachMark
	(I aMark sph3 (S .09 .05 .07))
)

(G stomachMarkGroup
	(I m1 stomachMark (T 0 0 .82) (R 45 0 0 1))
	(I m2 stomachMark (T .05 .05 .82) (R -45 0 0 1))
)

(G noseMark
	(I aMark sph6 (S .08 .05 .07))
)

(G totoro
    (I body sph1 (S .72 .7 .72))
		(I head sph1 (S .59 .6 .59) (T 0 .5 0))
		(I eye1 sph4 (S .16 .16 .2) (T -.217 .66 .4) (R 0 10 0 0 1))
		(I eye2 sph4 (S .16 .16 .2) (T .217 .66 .4))
		(I nose sph6 (S .1 .05 .1) (T 0 .56 .5))
		(I ear1 sph1 (S .15 .35 .15) (T 0 1.05 .16) (R 15 0 0 1))
		(I ear2 sph1 (S .15 .35 .15) (T .065 1.04 .15) (R -15 0 0 1))
		(I pupil sph5 (S .04 .04 .2) (T -.219 .66 .45) (R 0 10 0 0 1))
		(I pupil sph5 (S .04 .04 .2) (T .219 .66 .45))
		(I stomach sph2 (S .56 .56 .56) (T 0 0 .30))
		(I marks stomachMarkGroup (T -.02 .3 -.05))
		(I marks stomachMarkGroup (T .15 .3 -.07) (R -20 0 0 1))
		(I marks stomachMarkGroup (T -.19 .3 -.07) (R 20 0 0 1))
		#(I m3 stomachMark (T 0 0 .82))
)


(G scene
    (I cam1 cam)
    #(I dir1 dir (Xform       -.9304  0  .3665 0
    #                         .1444  -.9191  .3665 0
    #                         .3369  .3939 .8552 0
    #                         0  0   0 1))
    (I dir1 dir (Xform       -.8304  0  .1665 0
                             .1444  -.9191  .1665 0
                             .3369  .3939 .2552 0
                             0  0   0 1))
    (I amb1 amb)
    (I pt1 pt (T 0 0 -2.5))
		#(I pt1 pt (T 5 5 -19))
		(I pt1 pt (T 0 0 20))
    (I t totoro (T 0 -.2 -5) (R 0 0 0 1))
    (I sky ssph (S 2000 2000 .001) (T 0 0 -20))
    #(I sun ysph (S 3 3 .002) (T -5 5 -20))
    (I sl sunlt (T -5 5 -19))
)

(Render scene)


