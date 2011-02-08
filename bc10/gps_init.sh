#!/system/bin/sh

## GPIO175 : GPS_RESET
## GPIO140 : GPS_ON_OFF
## GPIO141 : GPS_VIO

SYSGPIO="/sys/class/gpio"
GPSRESET="${SYSGPIO}/gpio175"
GPSONOFF="${SYSGPIO}/gpio140"
GPSVIO="${SYSGPIO}/gpio141"

echo 175 > ${SYSGPIO}/export
echo 140 > ${SYSGPIO}/export
echo 141 > ${SYSGPIO}/export

echo out > ${GPSRESET}/direction
echo out > ${GPSONOFF}/direction
echo in  > ${GPSVIO}/direction

echo GPS_RESET `cat ${GPSRESET}/direction` `cat ${GPSRESET}/value`
echo GPS_ON_OFF `cat ${GPSONOFF}/direction` `cat ${GPSONOFF}/value`
echo GPS_VIO `cat ${GPSVIO}/direction` `cat ${GPSVIO}/value`

echo "Init GPS module"
echo 1 > ${GPSRESET}/value
sleep 1
echo 1 > ${GPSONOFF}/value
sleep 1
echo 0 > ${GPSONOFF}/value

echo GPS_RESET `cat ${GPSRESET}/direction` `cat ${GPSRESET}/value`
echo GPS_ON_OFF `cat ${GPSONOFF}/direction` `cat ${GPSONOFF}/value`
echo GPS_VIO `cat ${GPSVIO}/direction` `cat ${GPSVIO}/value`


