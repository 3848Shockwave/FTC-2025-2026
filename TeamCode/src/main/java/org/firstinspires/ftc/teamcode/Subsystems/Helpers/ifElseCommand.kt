package org.firstinspires.ftc.teamcode.Subsystems.Helpers

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.NullCommand

class ifElseCommand @JvmOverloads constructor(
    private val condition: () -> Boolean,
    private val trueCommand: Command,
    private val falseCommand: Command = NullCommand()
) : Command() {

    private lateinit var selectedCommand: Command

    override val isDone: Boolean get() = selectedCommand.isDone

    override fun start() {
        selectedCommand = if (condition()) trueCommand else falseCommand
        selectedCommand.start()
    }

    override fun update() = selectedCommand.update()

    override fun stop(interrupted: Boolean) = selectedCommand.stop(interrupted)
}