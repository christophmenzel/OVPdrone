import datetime

def microseconds():
  dt = datetime.datetime.now()
  return dt.microsecond + 1e6*dt.second + 6e7*dt.minute + 3.6e9*dt.hour
