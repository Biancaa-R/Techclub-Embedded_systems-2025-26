
echo "***************************************************************"
echo "installing required packages"
echo "***************************************************************"

python -m pip install pika --upgrade

echo "***************************************************************"
echo "For listing the active queues"
echo "***************************************************************"

rabbitmqctl.bat list_queues