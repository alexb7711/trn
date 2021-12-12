function filename = getDateTimeStringFilename( prefix )
temp = fix(clock);
%Extract Year
year = num2str(temp(1));
%Exract Month
month = num2str(temp(2));
if (length(month)<2)
    month = ['0',month];
end
%Extract Day
day = num2str(temp(3));
if (length(day)<2)
    day = ['0',day];
end
%Extract Hour
hour = num2str(temp(4));
if (length(hour)<2)
    hour = ['0',hour];
end
%Extract Minutes
minutes = num2str(temp(5));
if (length(minutes)<2)
    minutes = ['0',minutes];
end
%Extract Seconds
seconds = num2str(temp(6));
if (length(seconds)<2)
    seconds = ['0',seconds];
end
%Save the workspace
dateTimeString = [year '_' month '_' day '_' hour '_' minutes '_' seconds];
filename = [prefix,'_',dateTimeString];
end

