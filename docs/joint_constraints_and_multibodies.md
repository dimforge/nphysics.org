# Joint constraints and multibodies
One of the most appealing features of a physics engines is to simulate articulations. Articulation, or joints, allow the restriction of motion of one body part with regard to another. For example, one well-known joint is the ball-in-socket joint also known as the spherical joint: it allows one object to rotate freely with regard to the other but not to translate. This is typically used to simulate the shoulder of a ragdoll.

We can think of joint in several ways but first let's talk about the concept of **Degrees Of Freedom (DOF)**. In 3D, a rigid-body is capable of translating along the 3 coordinates axis $\mathbf{x}$, $\mathbf{y}$ and $\mathbf{z}$, and to rotate along those free axis as well. Therefore, a rigid-body is said to have  **6 translational DOF** and **3 rotational DOF**. We can also say a 3D rigid body has a total of **6 DOF**. The 2D case is similar but with less possibilities of movements: a 2D rigid body has 2 translational DOF and only 1 rotational DOF. The **relative DOF** of a body part wrt. another body part is the number of possible relative translations and rotations.

!!! Note
    The `BodyHandle::ground()` on the other hand cannot move at all. Therefore it has 0 DOFs. Moreover a [multibody](#multibodies) has a number of DOF that depend on its joints.

!!! Warning
    The number of DOF of a body or body part can be retrieved by `body.ndofs()`. The [status](/rigid_body_simulations_with_contacts/#rigid-body-statuses) of a body (dynamic, static, kinematic) does not affect the result of `body.ndofs()` even if technically a static and kinematic body should be seen as having no DOF (because no force can move them toward any direction). The `body.status_dependent_ndofs()` will take this body status into account and return 0 for static and kinematic bodies.

The goal of a joint is to reduce the number of DOF a body part has. For example, the aforementioned ball-in-socket (or spherical) joint removes all relative translations between two body parts. Therefore, it allows only 3 rotational DOF in 3D simulations or 1 rotational DOF in 2D simulations. Other joints exist allowing other combinations of relative DOF. Note that because there are more possible motions in 3D, some joints are not defined in 2D. This is illustrated by empty cells on the 2D column for joints that are not defined.

| Joint           | Allowed DOF in 2D | Allowed DOF in 3D |
|-----------------|-------------------|---------------|
| _Fixed joint_       | None           | None
| _Free joint_        | All            | All
| _Prismatic joint_   | 1 Translation  | 1 Translation |
| _Revolute joint_    | 1 Rotation     | 1 Rotation    |
| _Ball joint_        | 1 Rotation     | 3 Rotations |
| _Cartesian joint_   | 2 Translations | 3 Translations |
| _Planar joint_      |                | 2 Translations + 1 Rotation |
| _Cylindrical joint_ |                | 1 Translation + 1 Rotation (along the same axis) |
| _Pin-slot joint_    |                | 1 Translation + 1 Rotation (along different axises) |
| _Rectangular joint_ |                | 2 Translations |
| _Universal joint_   |                | 2 Rotations |

In 3D, a special _Helical joint_ also exists: it allows only one DOF which is a bit special as it is the combination of a rotation and a translation. In other word, a body part attached to the ground by an helical joint will only be able to translate and rotate simultaneously: any translation induce automatically a rotation and vice-versa.

-------

In practice, there are two main ways of modeling joints. Both are implemented by **nphysics** because each have very different advantages and limitations:

1. The **reduced-coordinates approach** encodes the reduction of degrees of motion directly into the equations of motion. For example, a 3D rigid body attached to the ground with a revolute joint will have its position encoded by only one variable: the rotation angle. Therefore, integrating its motion only changes this one variable and don't need additional forces or mathematical constraints to be generated. The clear advantage is that there is no way for the physics engine to apply any motion other than that single rotation to this boby, meaning there is no way the body shifts to a position that is not realistic, event if the dynamics solver don't converge completely.
2. The **constraints-based approach** (or full-coordinates approach) is the one most commonly available on other physics engines. Here, a 3D rigid body attached to the ground with a revolute joint will still have its position encoded by 6 variables (3 for translations and 3 for rotations). Then the integrator will add mathematical constraints to the dynamic system to ensure forces are applied to reduce the number DOF as imposed by joints. In practice, this means that the rigid body will break the joint constraint if the constrain solver does not converge completely.

This description shows only one aspect of the difference between the reduced-coordinates approach and the constraints-based approach. More generally, the reduced-coordinates approach favors accuracy while the constraints-based approach favors versatility. The following table compares the advantages and limitations of both approaches:

| Reduced-coordinates approach | Constraints-based approach |
|------------------------------|----------------------------|
| <font color="green">Joints cannot be violated.</font>                 | <font color="IndianRed">Joints can be violated if the solver does not converge.</font> |
| <font color="green">Moderately large time-step are possible.</font>   | <font color="IndianRed">Moderately large time-step may make the simulation explode.</font> |
| <font color="green">Large assembly are stable.</font>                                | <font color="IndianRed">Large assembly easily break without a large number of solver iterations.</font> |
| <font color="IndianRed">Adding/removing a join is slower.</font>          | <font color="green">Adding/removing a joint is fast.</font> |
| <font color="IndianRed">Joint forces are never computed, thus cannot be retrieved.</font>       | <font color="green">Joint forces are computed and can be retrieved.</font> |
| <font color="IndianRed">Topological restriction: body parts must be linked following a tree structure.</font> | <font color="green">The set of linked body parts can form any graph.</font> |

!!! Note "Which approach should I use?"
    The choice of approach **depends on the application**. For **robotics**, the **reduced-coordinates** approach is generally preferred because of its accuracy and ease of use, e.g., for control, inverse kinematics, etc.

    ---------

    **Video games** traditionally favor the **constraints-based** approach since most existing physics libraries either implement only this. Some other physics libraries also implement the reduced-coordinates approach as well but using the Featherstone algorithm which is extremely unstable in practice. In addition, the lower performance for addition and removal of reduced-coordinates joints may also preclude their use for some applications.

    ---------

    Simulating **closed loops** like for a necklace cannot be achieved with the reduced-coordinates approach only. However, it is possible to combine both approaches by using joint constraints only to close the loops. Refer to the [last section](#combining-multibodies-and-joint-constraints) for details.

The use of the reduced-coordinates approach is detailed in the [multibodies](#multibodies) section and demoed by the [Multibody joints](/demo_joints3/) demo. The constraints-based approach is detailed in the [joint constraints](#joint-constraints) section and demoed by the [Joint constraints](/demo_constraints3/) demo.

## Multibodies
Multibodies implement the reduced-coordinates approach. A multibody is a set of **multibody links** attached together by a **multibody joint**. Creating a multibody is implicitly done by adding multibody links to the scene. Indeed, two multibody links that are attached together (by a multibody joint) as considered part of the same multibody. Adding a multibody link to the world is very similar to adding a rigid body, but with more arguments passed to the `.add_multibody_link(...)` method which takes six arguments:

* `parent`: The other multibody link this is link is attached to. This can also be set to `BodyHandle::ground()` to indicate the multibody link is attached to the ground.
* `joint`: The multibody joint linking the newly created multibody link with its `parent`.
* `parent_shift`: The position of the joint wrt. `parent`, expressed in the local frame of `parent`.
* `body_shift`: The position of the newly created multibody link wrt. the joint, expressed in the local frame of the joint.
* `local_inertia`: The inertia of the multibody link on its reference frame.
* `local_center_of_mass`: The center of mass of the multibody link on its reference frame.


The following table summarizes the types corresponding to the joints mentioned on at the beginning of this chapter that can be used for the `joint` argument:

| Joint name | Multibody joint type on **nphysics** |
|------------|---------|
| _Fixed joint_       | [`FixedJoint`](/rustdoc/nphysics3d/joint/struct.FixedJoint.html)|
| _Prismatic joint_   | [`PrismaticJoint`](/rustdoc/nphysics3d/joint/struct.PrismaticJoint.html)  |
| _Revolute joint_    | [`RevoluteJoint`](/rustdoc/nphysics3d/joint/struct.RevoluteJoint.html)  |
| _Ball joint_        | [`BallJoint`](/rustdoc/nphysics3d/joint/struct.BallJoint.html)     |
| _Cartesian joint_   | [`CartesianJoint`](/rustdoc/nphysics3d/joint/struct.CartesianJoint.html) |
| _Planar joint_      | [`PlanarJoint`](/rustdoc/nphysics3d/joint/struct.PlanarJoint.html)     |
| _Cylindrical joint_ | [`CylindricalJoint`](/rustdoc/nphysics3d/joint/struct.CylindricalJoint.html)  |
| _Pin-slot joint_    | [`PinSlotJoint`](/rustdoc/nphysics3d/joint/struct.PinSlotJoint.html)   |
| _Rectangular joint_ | [`RectangularJoint`](/rustdoc/nphysics3d/joint/struct.RectangularJoint.html) |
| _Universal joint_   | [`UniversalJoint`](/rustdoc/nphysics3d/joint/struct.UniversalJoint.html)   |

!!! Note
    The first multibody link of a multibody is necessarily attached to `BodyHandle::ground()`. Note however that "attached" is a bit misleading here. If `joint` is set to an instance of `FreeJoint`, then this first multibody link will have all the possible degrees of freedom, making it completely free of perform any movement wrt. the ground.

!!! Warning
    The `FreeJoint` can be used only if `parent` is set to `BodyHandle::ground()` otherwise, the creation of the multibody link with `.add_multibody_link(...)` will panic.

You may refer to the [code](https://github.com/sebcrozet/nphysics/blob/master/examples3d/joints3.rs) of this [demo](/demo_joints3/) for concrete examples of joint constraint configurations.

## Joint constraints
Joint constraints implement the constraints-based approach. The following table summarizes the types corresponding to the joints mentioned on at the beginning of this chapter:

| Joint name | Joint constraint type on **nphysics** |
|------------|---------|
| _Fixed joint_       | [`FixedConstraint`](/rustdoc/nphysics3d/joint/struct.FixedConstraint.html)|
| _Prismatic joint_   | [`PrismaticConstraint`](/rustdoc/nphysics3d/joint/struct.PrismaticConstraint.html)  |
| _Revolute joint_    | [`RevoluteConstraint`](/rustdoc/nphysics3d/joint/struct.RevoluteConstraint.html)  |
| _Ball joint_        | [`BallConstraint`](/rustdoc/nphysics3d/joint/struct.BallConstraint.html)     |
| _Cartesian joint_   | [`CartesianConstraint`](/rustdoc/nphysics3d/joint/struct.CartesianConstraint.html) |
| _Planar joint_      | [`PlanarConstraint`](/rustdoc/nphysics3d/joint/struct.PlanarConstraint.html)     |
| _Cylindrical joint_ | [`CylindricalConstraint`](/rustdoc/nphysics3d/joint/struct.CylindricalConstraint.html)  |
| _Pin-slot joint_    | [`PinSlotConstraint`](/rustdoc/nphysics3d/joint/struct.PinSlotConstraint.html)   |
| _Rectangular joint_ | [`RectangularConstraint`](/rustdoc/nphysics3d/joint/struct.RectangularConstraint.html) |
| _Universal joint_   | [`UniversalConstraint`](/rustdoc/nphysics3d/joint/struct.UniversalConstraint.html)   |

A joint constraint is completely configured at its creation, and added to the world by the `world.add_constraint(constraint)` method. Each joint constraint require specific information for being constructed, but all roughly require:

1. The handles of the two body parts attached at each end of the joint. Handle of **any** type of body part is accepted. This includes rigid-bodies, `BodyHandle::ground()`, as well as a multibody link. Attaching a joint to a multibody link can be especially useful to handle complex assemblies with loops as described in the [next section](#combining-multibodies-and-joint-constraints).
2. The position of the joint endpoints with regard to each body part. A joint endpoint is often referred to as an **anchor** throughout the documentation of **nphysics**.

You may refer to the [code](https://github.com/sebcrozet/nphysics/blob/master/examples3d/constraints3.rs) of this [demo](/demo_constraints3/) for concrete examples of joint constraint configurations.


## Combining multibodies and joint constraints