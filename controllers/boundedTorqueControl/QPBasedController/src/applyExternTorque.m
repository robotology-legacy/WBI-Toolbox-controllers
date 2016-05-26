function TauExt = applyExternTorque(t, model)

    %t: simulation time

    %Apply an external torque to the joint
    if t >= model.TextIniTime && t <= model.TextEndTime
        TauExt = model.Text;
    else
        TauExt = 0;
    end
end