package org.xerosw.hid.mapping;

import org.xerosw.hid.IMappedGamepad;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class XboxMappedGamepad extends CommandXboxController implements IMappedGamepad {
    public XboxMappedGamepad(int port) {
        super(port);
    }
}
