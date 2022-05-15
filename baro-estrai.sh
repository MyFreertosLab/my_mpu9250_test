cat baro.sig |grep -e '^Baro: '|sed -e 's/Baro:..*A\[//g'|sed -e 's/\]..*$/;/g' > baro-4.sci
cat baro.sig |grep -e '^Baro: '|sed -e 's/Baro:..*VS\[//g'|sed -e 's/\]..*$/;/g' > baro-vs-4.sci
