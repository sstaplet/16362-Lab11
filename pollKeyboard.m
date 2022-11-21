function res = pollKeyboard()
    % pollKeyboard Waits until the callback says there is new data.
    % This routine is useful when you want to be able to capture an
    % respond to a keypress as soon as it arrives.
    % To use this, execute the following line:
    % set(fh,'KeyPressFcn',@keyboardEventListener);
    % before calling this function, where fh is a MATLAB figure handle.
    global keypressDataReady;
    global keypressKey;
    keyboardDataReadyLast = keypressDataReady;
    keypressDataReady = 0;

    if(keyboardDataReadyLast)
        res = keypressKey;
        disp('gotOne');
    else
        res = false;
    end
end