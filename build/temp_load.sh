cat_ret()
{
    cat /sys/class/thermal/thermal_zone0/temp
}

while true
do
time=$(date "+%Y-%m-%d-%H:%M:%S")
ret=`cat_ret`
echo "${time} $ret"
sleep 1
done
