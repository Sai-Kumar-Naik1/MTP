(define (problem drone-mission-001)

    (:domain swarm-drone-mission)

    ;; ============================================================
    ;; OBJECTS
    ;; ============================================================

    (:objects

        drone1 - drone

        ground-controller - controller

        source - location
        destination - location

        waypoint1 - location
        waypoint2 - location
    )


    ;; ============================================================
    ;; INITIAL STATE
    ;; ============================================================

    (:init

        ;; --------------------------------------------------------
        ;; Drone starting position
        ;; --------------------------------------------------------

        (at drone1 source)

        (source source)
        (destination destination)


        ;; --------------------------------------------------------
        ;; Route connectivity
        ;; --------------------------------------------------------

        (connected source destination)

        (connected source waypoint1)
        (connected waypoint1 waypoint2)
        (connected waypoint2 destination)

        (connected waypoint1 source)
        (connected waypoint2 waypoint1)
        (connected destination waypoint2)


        ;; --------------------------------------------------------
        ;; SAFE ROUTES
        ;; --------------------------------------------------------

        ;; Direct route is NOT safe because it crosses
        ;; the restricted area.

        (safe-route source waypoint1)
        (safe-route waypoint1 waypoint2)
        (safe-route waypoint2 destination)

        (safe-route waypoint1 source)
        (safe-route waypoint2 waypoint1)
        (safe-route destination waypoint2)


        ;; --------------------------------------------------------
        ;; RESTRICTED ROUTE
        ;; --------------------------------------------------------

        (restricted-route source destination)


        ;; --------------------------------------------------------
        ;; Drone system state
        ;; --------------------------------------------------------

        (gps-ok drone1)
        (communication-ok drone1)
        (drone-healthy drone1)


        ;; --------------------------------------------------------
        ;; Drone is initially not flying
        ;; --------------------------------------------------------


        ;; --------------------------------------------------------
        ;; Geographic coordinates
        ;; --------------------------------------------------------

        ;; Source
        (= (latitude source) 13.0827)
        (= (longitude source) 80.2707)

        ;; Destination
        (= (latitude destination) 13.0674)
        (= (longitude destination) 80.2376)

        ;; Waypoint 1
        (= (latitude waypoint1) 13.0830)
        (= (longitude waypoint1) 80.2250)

        ;; Waypoint 2
        (= (latitude waypoint2) 13.0650)
        (= (longitude waypoint2) 80.2250)


        ;; --------------------------------------------------------
        ;; DISTANCES
        ;; These values are calculated by the external
        ;; mission-generation program.
        ;; --------------------------------------------------------

        ;; Direct distance
        (= (distance source destination) 4300)

        ;; Alternative route
        (= (distance source waypoint1) 3900)
        (= (distance waypoint1 waypoint2) 2100)
        (= (distance waypoint2 destination) 1800)

        ;; Return distances
        (= (distance waypoint1 source) 3900)
        (= (distance waypoint2 waypoint1) 2100)
        (= (distance destination waypoint2) 1800)


        ;; --------------------------------------------------------
        ;; ENERGY REQUIRED
        ;; Example: energy units / percentage
        ;; --------------------------------------------------------

        (= (energy-required source destination) 45)

        (= (energy-required source waypoint1) 20)
        (= (energy-required waypoint1 waypoint2) 12)
        (= (energy-required waypoint2 destination) 10)

        (= (energy-required waypoint1 source) 20)
        (= (energy-required waypoint2 waypoint1) 12)
        (= (energy-required destination waypoint2) 10)


        ;; --------------------------------------------------------
        ;; BATTERY
        ;; --------------------------------------------------------

        (= (battery drone1) 85)

        (= (max-battery drone1) 100)

        ;; Drone should start emergency return when
        ;; battery becomes <= 30%

        (= (minimum-return-battery drone1) 30)

        ;; Battery consumed during hovering

        (= (hover-energy drone1) 2)


        ;; --------------------------------------------------------
        ;; HEALTH
        ;; --------------------------------------------------------

        (= (health drone1) 95)

        (= (minimum-health drone1) 40)


        ;; --------------------------------------------------------
        ;; Restricted area
        ;; --------------------------------------------------------

        ;; This is represented by the waypoint/route
        ;; preprocessing layer.

        (no-fly-zone waypoint1)
    )


    ;; ============================================================
    ;; GOAL
    ;; ============================================================

    (:goal

        (and

            ;; Drone must reach destination
            (mission-completed drone1)

            ;; Drone should be at destination
            (at drone1 destination)
        )
    )
)
