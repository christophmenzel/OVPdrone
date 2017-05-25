function val = dirt(rotor)
switch rotor
    case 'V'
        val = 1;
    case 'H'
        val = 1;
    case 'L'
        val = -1;
    case 'R'
        val = -1;
    otherwise
        val = NaN;
end