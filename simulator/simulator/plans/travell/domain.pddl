(define (domain swarm-drone-mission)

    (:requirements
        :strips
        :typing
        :numeric-fluents
        :negative-preconditions
        :conditional-effects
    )

    ;; ============================================================
    ;; TYPES
    ;; ============================================================

    (:types
        drone
        location
        waypoint
        controller
    )


    ;; ============================================================
    ;; PREDICATES
    ;; ============================================================

    (:predicates

        ;; Drone position
        (at ?d - drone ?l - location)

        ;; Mission locations
        (source ?l - location)
        (destination ?l - location)

        ;; Waypoint information
        (waypoint-location ?w - waypoint ?l - location)

        ;; Route connectivity
        (connected ?from - location ?to - location)

        ;; Safe route
        (safe-route ?from - location ?to - location)

        ;; Restricted / no-fly route
        (restricted-route ?from - location ?to - location)

        ;; Drone states
        (flying ?d - drone)
        (hovering ?d - drone)
        (mission-completed ?d - drone)

        ;; System health
        (gps-ok ?d - drone)
        (communication-ok ?d - drone)
        (drone-healthy ?d - drone)

        ;; Fault states
        (gps-lost ?d - drone)
        (communication-lost ?d - drone)
        (health-failure ?d - drone)

        ;; Ground-controller interaction
        (controller-notified ?d - drone)
        (controller-return ?d - drone)
        (controller-hover ?d - drone)
        (controller-sacrifice ?d - drone)

        ;; Notifications
        (battery-insufficient ?d - drone)
        (health-warning ?d - drone)
        (gps-warning ?d - drone)
        (communication-warning ?d - drone)

        ;; Emergency state
        (emergency-return ?d - drone)
        (sacrificed ?d - drone) 

        ;; Restricted area information
        (no-fly-zone ?w - waypoint)

        ;; Mission validity
        (route-planned ?d - drone)
        (distance-checked ?d - drone)
        (battery-checked ?d - drone)
    )


    ;; ============================================================
    ;; NUMERIC FUNCTIONS
    ;; ============================================================

    (:functions

        ;; Battery percentage
        (battery ?d - drone)

        ;; Maximum battery capacity
        (max-battery ?d - drone)

        ;; Battery required for a route
        (energy-required ?from - location ?to - location)

        ;; Distance between locations in meters
        (distance ?from - location ?to - location)

        ;; Drone health percentage
        (health ?d - drone)

        ;; Minimum health required for normal operation
        (minimum-health ?d - drone)

        ;; Minimum battery required for emergency return
        (minimum-return-battery ?d - drone)

        ;; Hover battery consumption
        (hover-energy ?d - drone)

        ;; Current route energy consumption
        (route-energy ?d - drone)

        ;; Latitude and longitude
        (latitude ?l - location)
        (longitude ?l - location)
    )


    ;; ============================================================
    ;; ACTION 1: CHECK DISTANCE
    ;; ============================================================

    (:action check-distance

        :parameters
            (?d - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (at ?d ?from)
                (connected ?from ?to)
            )

        :effect
            (distance-checked ?d)
    )


    ;; ============================================================
    ;; ACTION 2: CHECK BATTERY
    ;; ============================================================

    (:action check-battery

        :parameters
            (?d - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (at ?d ?from)
                (distance-checked ?d)
                (> (battery ?d)
                   (energy-required ?from ?to))
            )

        :effect
            (battery-checked ?d)
    )


    ;; ============================================================
    ;; ACTION 3: NOTIFY INSUFFICIENT BATTERY
    ;; ============================================================

    (:action notify-insufficient-battery

        :parameters
            (?d - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (at ?d ?from)
                (distance-checked ?d)
                (<= (battery ?d)
                    (energy-required ?from ?to))
            )

        :effect
            (battery-insufficient ?d)
    )


    ;; ============================================================
    ;; ACTION 4: PLAN SAFE ROUTE
    ;; ============================================================

    (:action plan-safe-route

        :parameters
            (?d - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (at ?d ?from)
                (connected ?from ?to)
                (safe-route ?from ?to)
                (battery-checked ?d)
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
            )

        :effect
            (route-planned ?d)
    )


    ;; ============================================================
    ;; ACTION 5: TRAVEL
    ;; ============================================================

    (:action travel

        :parameters
            (?d - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (at ?d ?from)
                (route-planned ?d)
                (safe-route ?from ?to)
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
                (> (battery ?d)
                   (energy-required ?from ?to))
            )

        :effect
            (and

                ;; Move drone
                (at ?d ?to)
                (not (at ?d ?from))

                ;; Drone is flying
                (flying ?d)

                ;; Consume battery
                (decrease
                    (battery ?d)
                    (energy-required ?from ?to)
                )

                ;; Remove old route plan
                (not (route-planned ?d))
            )
    )


    ;; ============================================================
    ;; ACTION 6: REACH DESTINATION
    ;; ============================================================

    (:action reach-destination

        :parameters
            (?d - drone
             ?dest - location)

        :precondition
            (and
                (at ?d ?dest)
                (destination ?dest)
            )

        :effect
            (and
                (mission-completed ?d)
                (not (flying ?d))
            )
    )


    ;; ============================================================
    ;; ACTION 7: GPS FAILURE
    ;; ============================================================

    (:action detect-gps-loss

        :parameters
            (?d - drone)

        :precondition
            (and
                (flying ?d)
                (not (gps-ok ?d))
            )

        :effect
            (and
                (gps-lost ?d)
                (gps-warning ?d)
                (not (flying ?d))
            )
    )


    ;; ============================================================
    ;; ACTION 8: COMMUNICATION FAILURE
    ;; ============================================================

    (:action detect-communication-loss

        :parameters
            (?d - drone)

        :precondition
            (and
                (flying ?d)
                (not (communication-ok ?d))
            )

        :effect
            (and
                (communication-lost ?d)
                (communication-warning ?d)
                (not (flying ?d))
            )
    )


    ;; ============================================================
    ;; ACTION 9: HEALTH FAILURE
    ;; ============================================================

    (:action detect-health-failure

        :parameters
            (?d - drone)

        :precondition
            (and
                (flying ?d)
                (< (health ?d)
                   (minimum-health ?d))
            )

        :effect
            (and
                (health-failure ?d)
                (health-warning ?d)
                (not (flying ?d))
            )
    )


    ;; ============================================================
    ;; ACTION 10: NOTIFY GROUND CONTROLLER
    ;; ============================================================

    (:action notify-ground-controller

        :parameters
            (?d - drone
             ?c - controller)

        :precondition
            (or
                (gps-lost ?d)
                (communication-lost ?d)
                (health-failure ?d)
            )

        :effect
            (controller-notified ?d)
    )


    ;; ============================================================
    ;; ACTION 11: GROUND CONTROLLER SAYS RETURN
    ;; ============================================================

    (:action controller-order-return

        :parameters
            (?d - drone)

        :precondition
            (and
                (controller-notified ?d)
                (controller-return ?d)
            )

        :effect
            (emergency-return ?d)
    )


    ;; ============================================================
    ;; ACTION 12: RETURN TO SOURCE
    ;; ============================================================

    (:action return-to-source

        :parameters
            (?d - drone
             ?current - location
             ?src - location)

        :precondition
            (and
                (at ?d ?current)
                (source ?src)
                (emergency-return ?d)
                (safe-route ?current ?src)
                (> (battery ?d)
                   (energy-required ?current ?src))
            )

        :effect
            (and
                (at ?d ?src)
                (not (at ?d ?current))
                (not (emergency-return ?d))

                (decrease
                    (battery ?d)
                    (energy-required ?current ?src)
                )
            )
    )


    ;; ============================================================
    ;; ACTION 13: CONTROLLER SAYS HOVER
    ;; ============================================================

    (:action controller-order-hover

        :parameters
            (?d - drone)

        :precondition
            (and
                (controller-notified ?d)
                (controller-hover ?d)
            )

        :effect
            (hovering ?d)
    )


    ;; ============================================================
    ;; ACTION 14: HOVER
    ;; ============================================================

    (:action hover

        :parameters
            (?d - drone)

        :precondition
            (and
                (hovering ?d)
                (> (battery ?d)
                   (minimum-return-battery ?d))
            )

        :effect
            (decrease
                (battery ?d)
                (hover-energy ?d)
            )
    )


    ;; ============================================================
    ;; ACTION 15: BATTERY REACHES RETURN LEVEL
    ;; ============================================================

    (:action initiate-low-battery-return

        :parameters
            (?d - drone)

        :precondition
            (and
                (hovering ?d)
                (<= (battery ?d)
                    (minimum-return-battery ?d))
            )

        :effect
            (and
                (emergency-return ?d)
                (not (hovering ?d))
            )
    )


    ;; ============================================================
    ;; ACTION 16: SACRIFICE / EMERGENCY LANDING
    ;; ============================================================

    (:action sacrifice-drone

        :parameters
            (?d - drone)

        :precondition
            (and
                (controller-notified ?d)
                (controller-sacrifice ?d)
            )

        :effect
            (and
                (sacrificed ?d)
                (not (flying ?d))
                (not (hovering ?d))
            )
    )

)

