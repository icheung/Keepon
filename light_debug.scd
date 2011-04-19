#cs184-ak
#cs184-fa

(Material sphere1mat
    (color .9 .9 .9)
    (ka .1) (kd .5) (ks .5) (ksp 150) (ksm 1) (kt 0) (ktn 1)
)

(Material sphere2mat
    (color .9 .4 .5)
    (ka .4) (kd .2) (ks .5) (ksp 20) (ksm 0) (kt 0) (ktn 1)
)

(Material sphere3mat
    (color .5 .9 .4)
    (ka .5) (kd .5) (ks .3) (ksp 4) (ksm .5) (kt 0) (ktn 1)
)

(Sphere sph1 (radius 2) (material sphere1mat))
(Sphere sph2 (radius 2.5) (material sphere2mat))
(Sphere sph3 (radius 0.25) (material sphere3mat))

(Light amb
    (type 3) (color .5 .2 .2)
)

(Light dir
    (type 0) (color .2 .2 1)
)

(Light pt
    (type 1) (color 1.39 .2 .2)
)

(Camera cam
    (perspective 1)
    (l -1) (r 1) (b -1) (t 1) (n 3) # no far plane 
)

(G threespheres
# we want the point light to cast the smaller sphere's shadow on the bigger sphere
    (I cam1 cam (T 0 0 0)) # camera above so it can view shadows
    (I amb1 amb)
    (I pt1 pt (T -1.5 0 -6))
		(I dir1 dir (Xform 0 0 -.5 0
                       0 0 0 0
                       0 0 0 0
                       0 0 0 0)) # points toward (.5,.5,-.5)
    (I s1 sph1 (T 2.5 0 -5)) # blue
    (I s3 sph3 (S 1 1 1) (T -0.5 0 -5)) # green
#    (I s2 sph2 (T 0 4.0 -25)) # red
)

(Render  threespheres)


