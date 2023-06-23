package com.msilimon.buoyancy

import com.badlogic.gdx.physics.box2d.Contact

interface Water {
    val buoyancyController: BuoyancyController

    fun step() {
        buoyancyController.step()
    }

    fun beginContact(contact: Contact) {
        buoyancyController.beginContact(contact)
    }

    fun endContact(contact: Contact) {
        buoyancyController.endContact(contact)
    }
}