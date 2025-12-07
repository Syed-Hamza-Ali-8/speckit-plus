import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures'; // Using English component for now
import Heading from '@theme/Heading';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className="relative h-screen flex items-center justify-center overflow-hidden">
      {/* Background Image with Overlay */}
      <img
        src="/img/hero-background.svg" // Placeholder image
        alt="فزیکل اے آئی اور ہیومنائیڈ روبوٹکس"
        className="absolute z-0 w-full h-full object-cover brightness-50"
      />
      <div className="absolute z-10 p-5 text-center text-white">
        <Heading as="h1" className="text-fluid-7xl md:text-fluid-8xl font-extrabold leading-tight mb-4 animate-fade-in-down text-gradient-to-r text-balance">
          {/* Site title from config should be translated via i18n */}
          {siteConfig.title}
        </Heading>
        <p className="text-fluid-2xl md:text-fluid-3xl text-white mb-8 animate-fade-in-up text-balance">
          {/* Tagline from config should be translated via i18n */}
          {siteConfig.tagline}
        </p>
        <div className="flex justify-center animate-fade-in">
          <Link
            className="
              bg-primary-500 hover:bg-primary-600
              text-white font-bold
              py-3 px-8 rounded-full shadow-lg
              transition duration-300 ease-in-out transform hover:scale-105
              text-lg
            "
            to="/docs/intro"> {/* Link to English doc for now, needs Urdu equivalent */}
            ٹیکسٹ بک دریافت کریں - سیکھنا شروع کریں 🚀
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`ہیلو ${siteConfig.title} سے`}
      description="یہ وضاحت میٹا ٹیگ میں جائے گی۔">
      <HomepageHeader />
      <main className="relative z-20">
        {/* About the Book Section */}
        <section className="py-20 bg-gray-50 dark:bg-gray-800">
          <div className="container mx-auto px-4 text-center">
            <Heading as="h2" className="text-fluid-4xl font-bold mb-4 text-gray-800 dark:text-white text-balance">
              "فزیکل اے آئی اور ہیومنائیڈ روبوٹکس" کے بارے میں
            </Heading>
            <p className="text-lg text-gray-600 dark:text-gray-300 max-w-3xl mx-auto mb-8">
              اس تبدیلی کی دنیا میں غوطہ لگائیں جہاں مصنوعی ذہانت فزیکل مجسمہ سے ملتی ہے۔ یہ ٹیکسٹ بک ہیومنائیڈ روبوٹس کے لیے اے آئی سسٹمز کو سمجھنے، ڈیزائن کرنے اور تعینات کرنے کے لیے ایک جامع گائیڈ فراہم کرتی ہے، جس میں بنیادی تصورات سے لے کر جدید نقلی اور حقیقی دنیا کی ایپلی کیشنز تک سب کچھ شامل ہے۔
            </p>
            <Link
              className="
                bg-secondary-500 hover:bg-secondary-600
                text-white font-bold
                py-3 px-8 rounded-full shadow-lg
                transition duration-300 ease-in-out transform hover:scale-105
              "
              to="/docs/intro"> {/* Link to English doc for now, needs Urdu equivalent */}
              مزید جانیں
            </Link>
          </div>
        </section>

        {/* Key Concepts Section */}
        <section className="py-20 bg-white dark:bg-gray-900">
          <div className="container mx-auto px-4 text-center">
            <Heading as="h2" className="text-fluid-4xl font-bold mb-4 text-gray-800 dark:text-white text-balance">
              اہم تصورات جن میں آپ مہارت حاصل کریں گے
            </Heading>
            <div className="grid grid-cols-1 md:grid-cols-3 gap-8 mt-10">
              <div className="p-6 rounded-lg shadow-md hover:shadow-xl transition-shadow duration-300 bg-gray-50 dark:bg-gray-800 animate-fade-in-up">
                <h3 className="text-2xl font-semibold mb-3 text-primary-500">ROS 2 اور روبوٹکس مڈل ویئر</h3>
                <p className="text-gray-600 dark:text-gray-300">جدید روبوٹکس کی ریڑھ کی ہڈی کو سمجھیں۔</p>
              </div>
              <div className="p-6 rounded-lg shadow-md hover:shadow-xl transition-shadow duration-300 bg-gray-50 dark:bg-gray-800 animate-fade-in-up delay-100">
                <h3 className="text-2xl font-semibold mb-3 text-secondary-500">اعلی درجے کی نقلی تکنیکیں</h3>
                <p className="text-gray-600 dark:text-gray-300">ڈیجیٹل ٹوئن اور مصنوعی ڈیٹا کی تخلیق میں مہارت حاصل کریں۔</p>
              </div>
              <div className="p-6 rounded-lg shadow-md hover:shadow-xl transition-shadow duration-300 bg-gray-50 dark:bg-gray-800 animate-fade-in-up delay-200">
                <h3 className="text-2xl font-semibold mb-3 text-[var(--ifm-color-tertiary)]">مجسم اے آئی اور رینفورسمنٹ لرننگ</h3>
                <p className="text-gray-600 dark:text-gray-300">فزیکل تعامل کے لیے ذہین ایجنٹ تیار کریں۔</p>
              </div>
            </div>
          </div>
        </section>
        {/* HomepageFeatures component will still be in English unless HomepageFeatures.ur.tsx is created */}
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
